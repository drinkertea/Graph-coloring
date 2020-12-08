#include "solver.h"
#include "models.h"

namespace solver
{

ColorizationResult Colorize(const Graph& graph)
{
    models::MainProblemModel model{ graph };
    return ColorizationResult();
}

};
