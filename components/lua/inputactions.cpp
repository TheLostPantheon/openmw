#include "inputactions.hpp"

#include <queue>
#include <set>

#include <components/debug/debuglog.hpp>

#include "luastate.hpp"
#ifdef __vita__
#include <algorithm>
#include <cstdio>

#include "scriptscontainer.hpp" // vitaLuaProfOn / vitaLuaNowUs

extern "C" void vitaBreadcrumb(const char* msg); // apps/openmw/vita/VitaInit.cpp
#endif

namespace LuaUtil
{
    namespace InputAction
    {
        namespace
        {
            std::string_view typeName(Type actionType)
            {
                switch (actionType)
                {
                    case Type::Boolean:
                        return "Boolean";
                    case Type::Number:
                        return "Number";
                    case Type::Range:
                        return "Range";
                    default:
                        throw std::logic_error("Unknown input action type");
                }
            }
        }

        MultiTree::Node MultiTree::insert()
        {
            size_t nextId = size();
            mChildren.push_back({});
            mParents.push_back({});
            return nextId;
        }

        bool MultiTree::validateTree() const
        {
            std::vector<bool> complete(size(), false);
            traverse([&complete](Node node) { complete[node] = true; });
            return std::find(complete.begin(), complete.end(), false) == complete.end();
        }

        template <typename Function>
        void MultiTree::traverse(Function callback) const
        {
            std::queue<Node> nodeQueue;
            std::vector<bool> complete(size(), false);
            for (Node root = 0; root < size(); ++root)
            {
                if (!complete[root])
                    nodeQueue.push(root);
                while (!nodeQueue.empty())
                {
                    Node node = nodeQueue.back();
                    nodeQueue.pop();

                    bool isComplete = true;
                    for (Node parent : mParents[node])
                        isComplete = isComplete && complete[parent];
                    complete[node] = isComplete;
                    if (isComplete)
                    {
                        callback(node);
                        for (Node child : mChildren[node])
                            nodeQueue.push(child);
                    }
                }
            }
        }

        bool MultiTree::multiEdge(Node target, const std::vector<Node>& source)
        {
            mParents[target].reserve(mParents[target].size() + source.size());
            for (Node s : source)
            {
                mParents[target].push_back(s);
                mChildren[s].push_back(target);
            }
            bool validTree = validateTree();
            if (!validTree)
            {
                for (Node s : source)
                {
                    mParents[target].pop_back();
                    mChildren[s].pop_back();
                }
            }
            return validTree;
        }

        namespace
        {
            bool validateActionValue(sol::object value, Type type)
            {
                switch (type)
                {
                    case Type::Boolean:
                        return value.get_type() == sol::type::boolean;
                    case Type::Number:
                        return value.get_type() == sol::type::number;
                    case Type::Range:
                        if (value.get_type() != sol::type::number)
                            return false;
                        double d = value.as<double>();
                        return 0.0 <= d && d <= 1.0;
                }
                throw std::invalid_argument("Unknown action type");
            }
        }

        void Registry::insert(const Info& info)
        {
            if (mIds.find(info.mKey) != mIds.end())
                throw std::domain_error("Action key \"" + info.mKey + "\" is already in use");
            if (info.mKey.empty())
                throw std::domain_error("Action key can't be an empty string");
            if (info.mL10n.empty())
                throw std::domain_error("Localization context can't be empty");
            if (!validateActionValue(info.mDefaultValue, info.mType))
                throw std::logic_error("Invalid value: \"" + LuaUtil::toString(info.mDefaultValue) + "\" for action \""
                    + info.mKey + "\"");
            Id id = mBindingTree.insert();
            mKeys.push_back(info.mKey);
            mIds[std::string(info.mKey)] = id;
            mInfo.push_back(info);
            mHandlers.push_back({});
            mBindings.push_back({});
            mValues.push_back(info.mDefaultValue);
        }

        std::optional<std::string> Registry::nextKey(std::string_view key) const
        {
            auto it = mIds.find(std::string(key));
            if (it == mIds.end())
                return std::nullopt;
            auto nextId = it->second + 1;
            if (nextId >= mKeys.size())
                return std::nullopt;
            return mKeys.at(nextId);
        }

        std::optional<Info> Registry::operator[](std::string_view actionKey)
        {
            auto iter = mIds.find(std::string(actionKey));
            if (iter == mIds.end())
                return std::nullopt;
            return mInfo[iter->second];
        }

        Registry::Id Registry::safeIdByKey(std::string_view key)
        {
            auto iter = mIds.find(std::string(key));
            if (iter == mIds.end())
                throw std::logic_error("Unknown action key: \"" + std::string(key) + "\"");
            return iter->second;
        }

        bool Registry::bind(
            std::string_view key, const LuaUtil::Callback& callback, const std::vector<std::string_view>& dependencies)
        {
            Id id = safeIdByKey(key);
            std::vector<Id> dependencyIds;
            dependencyIds.reserve(dependencies.size());
            for (std::string_view s : dependencies)
                dependencyIds.push_back(safeIdByKey(s));
            bool validEdge = mBindingTree.multiEdge(id, dependencyIds);
            if (validEdge)
                mBindings[id].push_back(Binding{
                    callback,
                    std::move(dependencyIds),
                });
            return validEdge;
        }

        sol::object Registry::valueOfType(std::string_view key, Type type)
        {
            Id id = safeIdByKey(key);
            Info info = mInfo[id];
            if (info.mType != type)
            {
                std::string message("Attempt to get value of type \"");
                message += typeName(type);
                message += "\" from action \"";
                message += key;
                message += "\" with type \"";
                message += typeName(info.mType);
                message += "\"";
                throw std::logic_error(message);
            }
            return mValues[id];
        }

        void Registry::update(double dt)
        {
#ifdef __vita__
            // [InputActions] (luaprof.txt): time per action key across its
            // bindings, value check and change handlers; logged every 300 calls.
            const bool prof = vitaLuaProfOn();
            static std::vector<unsigned long long> sUs;
            static std::vector<unsigned> sCalls;
            static unsigned sUpdates = 0;
            static unsigned long long sGlueUs = 0;
            unsigned long long nodeT0 = 0;
            const unsigned long long updT0 = prof ? vitaLuaNowUs() : 0;
            unsigned long long nodesUs = 0;
            if (prof && sUs.size() < mKeys.size())
            {
                sUs.resize(mKeys.size(), 0);
                sCalls.resize(mKeys.size(), 0);
            }
#endif
            std::vector<sol::object> dependencyValues;
            mBindingTree.traverse([&, this, dt](Id node) {
#ifdef __vita__
                if (prof)
                    nodeT0 = vitaLuaNowUs();
#endif
                sol::main_object newValue = mValues[node];
                std::vector<Binding>& bindings = mBindings[node];
                bindings.erase(std::remove_if(bindings.begin(), bindings.end(),
                                   [&](const Binding& binding) {
                                       if (!binding.mCallback.isValid())
                                           return true;

                                       dependencyValues.clear();
                                       for (Id parent : binding.mDependencies)
                                           dependencyValues.push_back(mValues[parent]);
                                       try
                                       {
                                           newValue = sol::main_object(
                                               binding.mCallback.call(dt, newValue, sol::as_args(dependencyValues)));
                                       }
                                       catch (std::exception& e)
                                       {
                                           if (!validateActionValue(newValue, mInfo[node].mType))
                                               Log(Debug::Error)
                                                   << "Error due to invalid value of action \"" << mKeys[node]
                                                   << "\"(\"" << LuaUtil::toString(newValue) << "\"): " << e.what();
                                           else
                                               Log(Debug::Error) << "Error in callback: " << e.what();
                                       }
                                       return false;
                                   }),
                    bindings.end());

                if (!validateActionValue(newValue, mInfo[node].mType))
                    Log(Debug::Error) << "Invalid value of action \"" << mKeys[node]
                                      << "\": " << LuaUtil::toString(newValue);
                if (mValues[node] != newValue)
                {
                    mValues[node] = sol::object(newValue);
                    std::vector<LuaUtil::Callback>& handlers = mHandlers[node];
                    handlers.erase(std::remove_if(handlers.begin(), handlers.end(),
                                       [&](const LuaUtil::Callback& handler) {
                                           if (!handler.isValid())
                                               return true;
                                           handler.tryCall(newValue);
                                           return false;
                                       }),
                        handlers.end());
                }
#ifdef __vita__
                if (prof && node < sUs.size())
                {
                    const unsigned long long us = vitaLuaNowUs() - nodeT0;
                    sUs[node] += us;
                    sCalls[node] += (unsigned)bindings.size();
                    nodesUs += us;
                }
#endif
            });
#ifdef __vita__
            if (!prof)
                return;
            sGlueUs += (vitaLuaNowUs() - updT0) - nodesUs; // traversal + setup outside nodes
            if (++sUpdates < 300)
                return;
            std::vector<std::size_t> order(sUs.size());
            for (std::size_t i = 0; i < order.size(); ++i)
                order[i] = i;
            std::sort(order.begin(), order.end(), [](std::size_t a, std::size_t b) { return sUs[a] > sUs[b]; });
            unsigned long long total = sGlueUs;
            for (unsigned long long us : sUs)
                total += us;
            char buf[600];
            int w = snprintf(buf, sizeof(buf), "[InputActions] %u actions, %.2f ms/update (outside nodes %.2f):",
                (unsigned)mKeys.size(), total / 1000.0 / sUpdates, sGlueUs / 1000.0 / sUpdates);
            for (std::size_t i = 0; i < order.size() && i < 12 && w > 0 && (std::size_t)w < sizeof(buf); ++i)
            {
                const std::size_t k = order[i];
                if (k < mKeys.size())
                    w += snprintf(buf + w, sizeof(buf) - w, " %s=%.0fus/%u", mKeys[k].c_str(),
                        sUs[k] / (double)sUpdates, sCalls[k] / sUpdates);
            }
            vitaBreadcrumb(buf);
            std::fill(sUs.begin(), sUs.end(), 0);
            std::fill(sCalls.begin(), sCalls.end(), 0);
            sGlueUs = 0;
            sUpdates = 0;
#endif
        }

        void Registry::clear(bool force)
        {
            std::vector<Info> infoToKeep;
            if (!force)
            {
                for (const Info& info : mInfo)
                    if (info.mPersistent)
                        infoToKeep.push_back(info);
            }
            mKeys.clear();
            mIds.clear();
            mInfo.clear();
            mHandlers.clear();
            mBindings.clear();
            mValues.clear();
            mBindingTree.clear();
            if (!force)
            {
                for (const Info& i : infoToKeep)
                    insert(i);
            }
        }
    }

    namespace InputTrigger
    {
        Registry::Id Registry::safeIdByKey(std::string_view key)
        {
            auto it = mIds.find(std::string(key));
            if (it == mIds.end())
                throw std::domain_error("Unknown trigger key \"" + std::string(key) + "\"");
            return it->second;
        }

        void Registry::insert(const Info& info)
        {
            if (mIds.find(info.mKey) != mIds.end())
                throw std::domain_error("Trigger key \"" + info.mKey + "\" is already in use");
            if (info.mKey.empty())
                throw std::domain_error("Trigger key can't be an empty string");
            if (info.mL10n.empty())
                throw std::domain_error("Localization context can't be empty");
            Id id = mIds.size();
            mIds[info.mKey] = id;
            mInfo.push_back(info);
            mHandlers.push_back({});
        }

        std::optional<Info> Registry::operator[](std::string_view key)
        {
            auto iter = mIds.find(std::string(key));
            if (iter == mIds.end())
                return std::nullopt;
            return mInfo[iter->second];
        }

        void Registry::registerHandler(std::string_view key, const LuaUtil::Callback& callback)
        {
            Id id = safeIdByKey(key);
            mHandlers[id].push_back(callback);
        }

        void Registry::activate(std::string_view key)
        {
            Id id = safeIdByKey(key);
            std::vector<LuaUtil::Callback>& handlers = mHandlers[id];
            handlers.erase(std::remove_if(handlers.begin(), handlers.end(),
                               [&](const LuaUtil::Callback& handler) {
                                   if (!handler.isValid())
                                       return true;
                                   handler.tryCall();
                                   return false;
                               }),
                handlers.end());
        }

        void Registry::clear(bool force)
        {
            std::vector<Info> infoToKeep;
            if (!force)
            {
                for (const Info& info : mInfo)
                    if (info.mPersistent)
                        infoToKeep.push_back(info);
            }
            mInfo.clear();
            mHandlers.clear();
            mIds.clear();
            if (!force)
            {
                for (const Info& i : infoToKeep)
                    insert(i);
            }
        }
    }
}
