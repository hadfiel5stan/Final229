module ProblemDefinition

# import Pkg; Pkg.add("POMDPs"); Pkg.add("QuickPOMDPs"); Pkg.add("Distributions")
using POMDPs, QuickPOMDPs, Distributions, LinearAlgebra
using POMDPTools: ImplicitDistribution, Uniform, Deterministic
include("winddirs.jl"); include("problemstate.jl")
using .winddirs, .boardstates

export RoutingProblem, id_to_location, location_to_id

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

function POMDPs.transition(pomdp::RoutingProblem, s, a)
    news = s
    
    if a == "up"
        news.y = news.y + 1
    elseif a == "down"
        news.y = news.y - 1
    elseif a == "right"
        news.x = news.x + 1
    else
        news.x = news.x - 1
    end

    return Deterministic(news)
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
    SUPERBAD = -100000000
    
    # If we go out of bounds that's a bad thing
    if sp.x > pomdp.width | sp.x < 1
        return SUPERBAD
    elseif sp.y > pomdp.height | sp.y < 1
        return SUPERBAD
    end

    # Set up our directional vector
    aval = (findall(x->x==a, ["up", "down", "left", "right"])[1] - 1)*pi/2
    avec = [cos(aval), sin(aval)]
    
    # Extract the wind vector
    wcond = s.windconds[location_to_id(s.x, s.y, pomdp)]
    wvec = wcond.dirvec

    # Figure out how much of the wind will be a problem
    factor = dot(avec, wvec)
    # Calculate our speed in the movement direction
    finalspeed = pomdp.cruisespeed + factor*wcond.speed

    # Cost is the burnt fuel
    c = -pomdp.burnrate*(pomdp.spacing/finalspeed)

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
    if s.x == p.endx & s.y == p.endy
        return(true)
    end

    if s.x > p.width | s.x < 1 | s.y > p.height | s.y < 1
        return(true)
    end

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

end