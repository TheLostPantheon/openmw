#ifndef OPENMW_COMPONENTS_ESM3_ACTORIDCONVERTER_H
#define OPENMW_COMPONENTS_ESM3_ACTORIDCONVERTER_H

#include <map>
#include <vector>

#include "refnum.hpp"

namespace ESM
{
    class ActorIdConverter
    {
        std::vector<std::pair<ESM::RefNum&, int>> mToConvert;

    public:
        std::map<int, ESM::RefNum> mMappings;
        std::vector<int> mGraveyard;

        void apply();

        void convert(ESM::RefNum& refNum, int actorId);

        /// Entries reference parsed store fields; see resolveFrom.
        std::size_t pendingCount() const { return mToConvert.size(); }
        /// Resolve from idx; false = pending remain, keep store alive.
        bool resolveFrom(std::size_t idx);
    };
}

#endif
