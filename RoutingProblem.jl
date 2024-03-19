module ProblemDefinition

# import Pkg; Pkg.add("POMDPs"); Pkg.add("QuickPOMDPs"); Pkg.add("Distributions")
using POMDPs, QuickPOMDPs, Distributions, LinearAlgebra, POMDPSimulators, ARDESPOT
using POMDPTools: ImplicitDistribution, Uniform, Deterministic
include("winddirs.jl"); include("problemstate.jl")
using .winddirs, .boardstates

struct RoutingProblemBounds
    dummy::String
end

struct RoutingProblem <: POMDP{boardstates.problemstate, String, boardstates.observation}
    width::Int64
    height::Int64
    upperwinds::Matrix{Int64}
    cruisespeed::Int64
    spacing::Int64
    burnrate::Float64
    startx::Int64
    starty::Int64
    endx::Int64
    endy::Int64
end

POMDPs.states(pomdp::RoutingProblem) = boardstates.statespaceiter(pomdp.width, pomdp.height)

POMDPs.actions(pomdp::RoutingProblem) = ["up", "down", "left", "right"]
POMDPs.actionindex(pomdp::RoutingProblem, a) = findall(x->x==a, ["up", "down", "left", "right"])[1]

function POMDPs.observations(pomdp::RoutingProblem)
    os = []
    for i in 1:pomdp.width
        for j in 1:pomdp.height
            for k in 1:Int64(winddirs.MAX_DIR/winddirs.INC_DIR + 1)
                for l in 1:Int64(winddirs.MAX_SPEED/winddirs.INC_SPEED)
                    # loops are x, y, direction, speed
                    push!(os, boardstates.createobservationfromindices(i, j, k, l))
                end
            end
        end
    end
    return os
end
function POMDPs.obsindex(pomdp::RoutingProblem, o)
    countdir = Int64(winddirs.MAX_DIR/winddirs.INC_DIR + 1)
    countspeed = Int64(winddirs.MAX_SPEED/winddirs.INC_SPEED)

    dirind = Int64(o.windcond.dir.value/winddirs.INC_DIR) + 1
    speedind = Int64(o.windcond.speed/winddirs.INC_SPEED)

    i = speedind + countspeed*(dirind - 1) + countspeed*countdir*(o.y - 1) + countspeed*countdir*pomdp.height*(o.x - 1)
    
    return i
end

function POMDPs.transition(pomdp::RoutingProblem, s::boardstates.problemstate, a)
    if POMDPs.isterminal(pomdp, s)
        return Deterministic(s)
    end
    
    x = s.x
    y = s.y
    
    if a == "up"
        y = y + 1
    elseif a == "down"
        y = y - 1
    elseif a == "right"
        x = x + 1
    else
        x = x - 1
    end
    # So far punishing going out of bounds is either tempting
    # or messing with the reward function so for now, you can't go out of bounds
    x = clamp(x, 1, pomdp.width)
    y = clamp(y, 1, pomdp.height)

    return Deterministic(boardstates.problemstate(x, y, s.windconds))
end

# As of now, an observation is a certain representation of winds at that position
function POMDPs.observation(pomdp::RoutingProblem, sp::ProblemDefinition.boardstates.problemstate)
    os = []
    for (i, wc) in enumerate(sp.windconds)
        x, y = id_to_location(i, pomdp)
        push!(os, boardstates.observation(x, y, wc))
    end

    return Uniform(os)
end

function POMDPs.reward(pomdp::RoutingProblem, s, a, sp)
    SUPERBAD = -200000
    SUPERGOOD = 0
    
    # If we go out of bounds that's a bad thing
    if sp.x > pomdp.width || sp.x < 1 || s.x > pomdp.width || s.x < 1
        return SUPERBAD
    elseif sp.y > pomdp.height || sp.y < 1 || s.y > pomdp.height || s.y < 1
        return SUPERBAD
    end

    # Set up our directional vector
    aval = (findall(x->x==a, ["up", "right", "down", "left"])[1] - 1)*pi/2
    avec = [sin(aval), cos(aval)]
    
    # Extract the wind vector
    wcond = s.windconds[location_to_id(s.x, s.y, pomdp)]
    wvec = wcond.dirvec

    # Figure out how much of the wind will be a problem
    factor = dot(avec, wvec)
    # Calculate our speed in the movement direction
    finalspeed = pomdp.cruisespeed + factor*wcond.speed

    # Cost is the burnt fuel
    c = -pomdp.burnrate*(pomdp.spacing/finalspeed)

    if sp.x == pomdp.endx && sp.y == pomdp.endy
        c = c + SUPERGOOD
    end

    return c
end

POMDPs.discount(p::RoutingProblem) = 0.95

function POMDPs.initialstate(p::RoutingProblem)
    ImplicitDistribution() do rng
        perturbs = rand(rng, -1:1, (2, p.width*p.height))
        
        perturbs[1, :] = perturbs[1, :]*winddirs.INC_DIR
        perturbs[2, :] = perturbs[2, :]*winddirs.INC_SPEED

        newwinds = p.upperwinds + perturbs

        newwinds[1, :] = mod.(newwinds[1, :], winddirs.MAX_DIR)
        newwinds[2, :] = clamp.(newwinds[2, :], winddirs.MIN_SPEED, winddirs.MAX_SPEED)
    
        return boardstates.createstate(p.startx, p.starty, newwinds)
    end
end

function POMDPs.isterminal(p::RoutingProblem, s::boardstates.problemstate)
    if s.x == p.endx && s.y == p.endy
        return(true)
    end

    # if s.x > p.width || s.x < 1 || s.y > p.height || s.y < 1
    #     return(true)
    # end

    return false
end

function id_to_location(id, pomdp::RoutingProblem)
    x = mod(id - 1, pomdp.width) + 1
    y = fld(id, pomdp.width) + 1

    return(x, y)
end

function id_to_location(id, w::Int64)
    x = mod(id - 1, w) + 1
    y = fld(id, w) + 1

    return(x, y)
end

function location_to_id(x, y, pomdp::RoutingProblem)
    id = (y-1)*pomdp.width + x

    return(id)
end

function location_to_id(x, y, w::Int64)
    id = (y-1)*w + x

    return(id)
end

function AStarRollout(p::RoutingProblem, s::boardstates.problemstate, h, steps)
    localPlanner = Astarplanner()

    path, policy = AstarPlan(p, localPlanner, [s.x, s.y])

    p2 = RoutingProblem(p.width,p.height,p.upperwinds,p.cruisespeed, p.spacing, p.burnrate, s.x, s.y, p.endx, p.endy)

    hist = sim(p2) do o
        ret = policy[1]
        deleteat!(policy, 1)
        return ret
    end

    rfinal = 0

    rfinal = [r for (s,a,sp, o, r) in hist]

    return sum(rfinal)
end


mutable struct Astarplanner
    name::String
end

function Astarplanner(name = "myAstar")
    Astarplanner(name)
end

function AstarPlan(p::ProblemDefinition.RoutingProblem, planner::Astarplanner, startpt=nothing )
    if isnothing(startpt)
        startpt = [p.startx, p.starty]
    end
    donept = [p.endx, p.endy]

    openset = Set{Vector{Int64}}()
    push!(openset, startpt)

    camefrom = Dict{Vector{Int64}, Union{Nothing, Vector{Int64}}}(startpt => nothing)
    camefrom[donept] = startpt

    costtocome = Dict(startpt => 0.0)

    costtogo = Dict(startpt => h(startpt, p))

    while length(openset) > 0
        currscores = [[costtogo[p]; p] for p in openset]
        currpt = Int.(currscores[findmin(currscores)[2]][2:3])

        if norm(currpt - donept) < 0.1
            return(reconstructPath(currpt, camefrom))
        end

        delete!(openset, currpt)

        for i in 1:4
            theta = i * pi/2
            perturbvec = Int.(round.([sin(theta), cos(theta)]))

            neigh = currpt + perturbvec

            tentativec2c = costtocome[currpt] + AstarCost(currpt, neigh, p)

            # This could fail either if we found a better path or if we haven't been there yet
            try
                if tentativec2c > costtocome[neigh]
                    continue
                end
            catch KeyError
                
            end

            camefrom[neigh] = currpt
            costtocome[neigh] = tentativec2c
            costtogo[neigh] = tentativec2c + h(neigh, p)

            if neigh ∉ openset
                push!(openset, neigh)
            end
        end
    end
end

# Estimates lower bound cost to go, basically straight line distance with tailwind
function h(p1::Vector{Int64}, p::ProblemDefinition.RoutingProblem)

    d = norm([p.endx, p.endy] - p1)

    speed = p.cruisespeed + winddirs.MAX_SPEED

    return(p.burnrate*(d*p.spacing)/speed)
end

function AstarCost(p1::Vector{Int64}, p2::Vector{Int64}, p::ProblemDefinition.RoutingProblem)
    if p1[1] < 1 || p1[1] > p.width || p1[2] < 1 || p1[2] > p.height
        return 100000
    end
    
    dvec = p2 - p1
    wdir, wspeed = p.upperwinds[:, location_to_id(p1[1], p1[2], p)]
    windvec = [sind(wdir), cosd(wdir)]

    factor = dot(dvec, windvec)

    speed = p.cruisespeed + factor*wspeed

    c = p.burnrate*p.spacing/speed

    return c
end

function reconstructPath(curr, cameFrom)
    fullp = []
    as = []
    actionmap = Dict([1, 0] => "right", [-1, 0] => "left", [0, 1] => "up", [0, -1] => "down")

    while curr ∈ keys(cameFrom)
        push!(fullp, curr)

        try
            dvec = curr - cameFrom[curr]
            push!(as, actionmap[dvec])
        catch
        end

        curr = cameFrom[curr]
    end 
    
    return (reverse(fullp), reverse(as))
end

end