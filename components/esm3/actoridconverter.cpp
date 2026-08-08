#include "actoridconverter.hpp"

namespace ESM
{
    void ActorIdConverter::apply()
    {
        for (auto& [refNum, actorId] : mToConvert)
        {
            auto it = mMappings.find(actorId);
            if (it == mMappings.end())
                refNum = {};
            else
                refNum = it->second;
        }
    }

    bool ActorIdConverter::resolveFrom(std::size_t idx)
    {
        // pair<RefNum&,int> is not assignable; rebuild to filter.
        std::vector<std::pair<ESM::RefNum&, int>> kept;
        kept.reserve(mToConvert.size());
        bool allResolved = true;
        for (std::size_t i = 0; i < mToConvert.size(); ++i)
        {
            auto& [refNum, actorId] = mToConvert[i];
            if (i < idx)
            {
                kept.emplace_back(refNum, actorId);
                continue;
            }
            auto it = mMappings.find(actorId);
            if (it != mMappings.end())
                refNum = it->second; // resolved now; entry dropped
            else
            {
                kept.emplace_back(refNum, actorId);
                allResolved = false;
            }
        }
        mToConvert.swap(kept);
        return allResolved;
    }

    void ActorIdConverter::convert(ESM::RefNum& refNum, int actorId)
    {
        if (actorId == -1)
        {
            refNum = {};
            return;
        }
        auto it = mMappings.find(actorId);
        if (it == mMappings.end())
        {
            mToConvert.emplace_back(refNum, actorId);
            return;
        }
        refNum = it->second;
    }
}
