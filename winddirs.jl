module winddirs

export dirs, windcond, init_windcond

# Beware, eveen with this reduced problem, the state space is immense...
const MAX_SPEED = 20
const MIN_SPEED = 5
const INC_SPEED = 5

const INC_DIR = 45
const MAX_DIR = 315

@enum dirs begin
    d0 = 0
    d10 = 10
    d20 = 20
    d30 = 30
    d40 = 40
    d45 = 45
    d50 = 50
    d60 = 60
    d70 = 70
    d80 = 80
    d90 = 90
    d100 = 100
    d110 = 110
    d120 = 120
    d130 = 130
    d135 = 135
    d140 = 140
    d150 = 150
    d160 = 160
    d170 = 170
    d180 = 180
    d190 = 190
    d200 = 200
    d210 = 210
    d220 = 220
    d225 = 225
    d230 = 230
    d240 = 240
    d250 = 250
    d260 = 260
    d270 = 270
    d280 = 280
    d290 = 290
    d300 = 300
    d310 = 310
    d315 = 315
    d320 = 320
    d330 = 330
    d340 = 340
    d350 = 350
end

mutable struct windcond
    dir::dirs
    dirvec::Vector
    speed::Int64
end

init_windcond = function(d::dirs, s::Int64)
    if s < MIN_SPEED
        s = MIN_SPEED
    elseif s > MAX_SPEED
        s = MAX_SPEED
    end

    return windcond(d, [sind(Int(d)), cosd(Int(d))], s)
end

init_windcond = function(d::Int64, s::Int64)
    if s < MIN_SPEED
        s = MIN_SPEED
    elseif s > MAX_SPEED
        s = MAX_SPEED
    end

    if d ∉ keys(Base.Enums.namemap(dirs))
        throw("Direction must be in 10 degree increments: "*string(d))
    end
    
    return windcond(dirs(d), [sind(Int(d)), cosd(Int(d))], s)
end

end