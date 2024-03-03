include("winddirs.jl"); include("problemstate.jl"); include("RoutingProblem.jl")
using .winddirs, .boardstates, .ProblemDefinition
using POMDPs, MCVI
using Random: MersenneTwister


function MCVI.lower_bound(lb::ProblemDefinition.RoutingProblemBounds, 
                            p::ProblemDefinition.RoutingProblem, s::ProblemDefinition.boardstates.problemstate)
    d = minimum([
        s.x,
        s.y,
        p.width - s.x,
        p.height - s.y
    ])
    SUPERBAD = -100000000
    return SUPERBAD*ProblemDefinition.discount(p)^(d)
end

function MCVI.upper_bound(ub::ProblemDefinition.RoutingProblemBounds,
                            p::ProblemDefinition.RoutingProblem, s::ProblemDefinition.boardstates.problemstate)
    L1 = abs(s.x - p.endx) + abs(s.y - p.endy)

    speed = p.cruisespeed + winddirs.MAX_SPEED

    return -(L1/speed)*p.burnrate*p.spacing*ProblemDefinition.discount(p)^(L1)
end

w = 10
h = 5
uwinds = zeros((2, w*h))

for i in 1:w*h
    if ProblemDefinition.id_to_location(i, w)[2] < 5
        uwinds[:, i] = [315, 10]
    else
        uwinds[:, i] = [225, 15]
    end
end


sim = MCVISimulator(MersenneTwister(1))
b = ProblemDefinition.RoutingProblemBounds("Dummy")

s1 = ProblemDefinition.RoutingProblem(w, h, uwinds, 100, 2500, 6.4, 1, 3, 10, 5)
solver = MCVISolver(sim, nothing, 1, 100, 8, 500, 1000, 5000, 50, b, b)

policy = POMDPs.solve(solver, s1)