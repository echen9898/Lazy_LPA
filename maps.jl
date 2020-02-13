
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Standard Object Map ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function gen_object_lists(object_defs::Dict{String, Any})::Dict{String, Any}
    object_lists = Dict{String, Any}()
    for pair ∈ object_defs
        shape = pair[1]
        center = pair[2][1]
        radius = pair[2][4]
        if shape[1] == 's' # square
            object_lists[shape] = vcat(center, gen_square(center, radius))
        elseif shape[1] == 'p' # plus sign
            object_lists[shape] = vcat(center, gen_plus(center, radius))
        end
    end
    return object_lists
end

function move_objects(w::Int64, h::Int64, object_defs::Dict{String, Any})::Dict{String, Any}
    for pair ∈ object_defs
        shape = pair[1]
        center = pair[2][1]
        speed = pair[2][2]
        direction = pair[2][3]
        radius = pair[2][4]

        if direction == 0 # left
            new_center = (center[1]-speed, center[2])
            margin = center[1]-speed-radius
            if margin < 0
                direction = 2
                new_center = (center[1]-speed+abs(margin), center[2])
            end
        elseif direction == 1 # up
            new_center = (center[1], center[2]+speed)
            margin = (center[2]+speed+radius)-h
            if margin > 0
                direction = 3
                new_center = (center[1], center[2]+speed-margin)
            end
        elseif direction == 2 # right
            new_center = (center[1]+speed, center[2])
            margin = (center[1]+speed+radius)-w # distance from edge
            if margin > 0
                direction = 0
                new_center = (center[1]+speed-margin, center[2])
            end
        elseif direction == 3 # down
            new_center = (center[1], center[2]-speed)
            margin = center[2]-speed-radius
            if margin < 0
                direction = 1
                new_center = (center[1], center[2]-speed+abs(margin))
            end
        end

        object_defs[shape] = [new_center, speed, direction, radius]
    end
    return object_defs
end

function flatten_objects(object_lists::Dict{String, Any})::Array{Tuple{Int64, Int64}}
    obstacles = Array{Tuple{Int64, Int64}}[]
    for pair ∈ object_lists
        obstacles = [obstacles ; pair[2]]
    end

    return collect(Set(obstacles))
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Assorted Generators ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function gen_square(center::Tuple{Int64, Int64}, layers::Int64)::Array{Tuple{Int64, Int64}}
    occupied = Set{Tuple{Int64, Int64}}()
    deltas = collect(-layers:layers)
    for d_x ∈ deltas
        for d_y ∈ deltas
            push!(occupied, (center[1]+d_x, center[2]+d_y))
        end
    end
    return collect(occupied)
end

function gen_plus(center::Tuple{Int64, Int64}, arm_length::Int64)::Array{Tuple{Int64, Int64}}
    occupied = Set{Tuple{Int64, Int64}}()
    deltas = collect(-arm_length:arm_length)
    for d ∈ deltas
        push!(occupied, (center[1]+d, center[2]))
        push!(occupied, (center[1], center[2]+d))
    end
    return collect(occupied)
end

function generate_random_obstacles(start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, 
                                    w::Int64, h::Int64, num_obstacles::Int64)::Array{Tuple{Int64, Int64}}

    obstacles = Set{Tuple{Int64, Int64}}()
    while length(obstacles) < num_obstacles
        o = (rand(1:w), rand(1:h))
        o ∉ [start, goal] ? push!(obstacles, o) : continue
    end
    return collect(obstacles)
end

