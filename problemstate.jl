module boardstates

include("winddirs.jl")

using .winddirs

export problemstate

mutable struct problemstate
    x::Int64
    y::Int64
    windconds::Vector{winddirs.windcond}
end

struct statespaceiter
    w::Int64
    h::Int64
end

struct observation
    x::Int64
    y::Int64
    windcond::winddirs.windcond
end

function createobservationfromindices(x, y, didx, sidx)
    return boardstates.observation(x, y, winddirs.init_windcond(Int64(winddirs.INC_DIR*(didx - 1)), 
                Int64(winddirs.INC_SPEED*sidx)))
end

# Top is directions, bottom is Speeds
function createstate(x, y, conds)
    # Preallocate vector of winddirs
    cs = Vector{winddirs.windcond}(undef, size(conds)[2])

    # Build each winddir type
    for i in 1:size(conds)[2]
        cs[i] = winddirs.init_windcond(conds[1,i], conds[2,i])
    end
    # Return the full struct
    return problemstate(x, y, cs)
end

# Required iterator functions to allow us to use statespaceiter
function Base.iterate(iter::statespaceiter)
    if iter.h == 0 | iter.w == 0
        return nothing

    else
        istate = zeros(Int64, (2, 1 + iter.h * iter.w))
        istate[2, 2:end] .= winddirs.MIN_SPEED
    
        return (createstate(0, 0, istate[:, 2:end]), istate)

    end
end

# Takes a statespace iter and returns the next state
function Base.iterate(iter::statespaceiter, curr)
    # We'll call the last state just the max everything
    if sum(curr) == iter.h + iter.w + 
                    (iter.h * iter.w * winddirs.MAX_SPEED) + 
                    (iter.h * iter.w * winddirs.MAX_DIR)

        return nothing
    # Else we basically need to increment a weird mixed base number
    else
        ifound = -1
        jfound = -1
        # In this first area, we've maxed out a windspeed or direction
        for i in (iter.h * iter.w + 1):-1:2
            if curr[2, i] != winddirs.MAX_SPEED
                ifound = i
                jfound = 2
                break
            elseif curr[1, i] != winddirs.MAX_DIR
                ifound = i
                jfound = 1
                # Reset the speed beneath
                curr[2, i] = winddirs.MIN_SPEED
                break
            end
        end
        if ifound == -1
            if curr[2, 1] != iter.h
                ifound = 1
                jfound = 2
                
            elseif curr[1,1] != iter.w
                ifound = 1
                jfound = 1
                # Reset the y coordinate beneath
                curr[2, 2] = 1
            end
        end

        # Increment properly
        if ifound == 1
            curr[jfound, ifound] = curr[jfound, ifound] + 1
        elseif jfound == 1
            curr[jfound, ifound] = curr[jfound, ifound] + winddirs.INC_DIR
        else
            curr[jfound, ifound] = curr[jfound, ifound] + winddirs.INC_SPEED
        end
        # Reset speeds dirs below
        curr[1, ifound+1:end] .= 0
        curr[2, ifound+1:end] .= winddirs.MIN_SPEED
    end
    return (createstate(curr[1,1], curr[2,1], curr[:, 2:end]), curr)
end

function Base.length(iter::statespaceiter)
    ndirs = winddirs.MAX_DIR/winddirs.INC_DIR + 1
    nspeeds = (winddirs.MAX_SPEED - winddirs.MIN_SPEED)/winddirs.INC_SPEED + 1
    nlocs = iter.w*iter.h
    return nlocs*(ndirs^(nlocs)*nspeeds^(nlocs))
end


end