include("RoutingProblem.jl")
using .ProblemDefinition
using POMDPs, MCVI, BasicPOMCP, POMDPSimulators, POMCPOW, ARDESPOT, ParticleFilters
using Random: MersenneTwister
using DelimitedFiles


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

function DirectBounds(p::ProblemDefinition.RoutingProblem, b::ScenarioBelief)
    x, y = particle(b,1).x, particle(b,1).y

    l = (abs(p.endx - x) + abs(p.endy - y))*p.spacing

    speedwithmaxheadwind = p.cruisespeed - ProblemDefinition.winddirs.MAX_SPEED
    speedwithmaxtailwind = p.cruisespeed + ProblemDefinition.winddirs.MAX_SPEED

    upper = l*p.burnrate/speedwithmaxtailwind
    lower = l*p.burnrate/speedwithmaxheadwind

    return (-lower, -upper)
end

w = 15
h = 10
uwinds = zeros((2, w*h))

for i in 1:w*h
    if ProblemDefinition.id_to_location(i, w)[2] == 5
        uwinds[:, i] = [0, 10]
    else
        uwinds[:, i] = [0, 15]
    end
end



s1 = ProblemDefinition.RoutingProblem(w, h, uwinds, 100, 2500, 6.4, 1, 5, 15, 7)
Astaryes = false
@elapsed for i in 1:20

    if Astaryes
        planner = ProblemDefinition.Astarplanner()
        plan, aplan = ProblemDefinition.AstarPlan(s1, planner)

        hist = sim(s1) do o
            ret = aplan[1]
            deleteat!(aplan, 1)
            return ret
        end
        tag = "_Astar"
    else

        solver = DESPOTSolver(bounds = DirectBounds)
        planner = POMDPs.solve(solver, s1)

        hr = HistoryRecorder(max_steps = 40)
        hist = simulate(hr, s1, planner)
        tag = "DESPOT"
    end

    open("results_scenario2"*tag*".csv","a") do f
        for (s, a, sp, o, r) in hist
            writedlm(f, [s.x s.y a r ""], ",")
            #println(string(s.x)*", "*string(s.y))
            @show s.x, s.y, a, r
            println("---------------------------")
        end
        writedlm(f, ["" "" "" "" discounted_reward(hist)], ",")
        writedlm(f, ["" "" "" "" ""], ",")
    end

end
