
using DataStructures
using TimerOutputs
using StatsBase
include("utils.jl")
include("plot.jl")
include("maps.jl")

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

w = 50
h = 100
start = (1, 1)
goal = (40, 100)
robot_speed = 2 # minimum of 2
object_defs = Dict{String, Any}( # center_coord, speed, direction (0=left, 1=up, 2=right, 3=down), radius
    "square_1" => [(9, 15), 5, 2, 2],
    "plus_1" => [(14, 30), 1, 1, 4]
    )
plan = Array{Tuple{Int64, Int64}}[]
timelimit = 1000
to = TimerOutput() # timers


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ A* Planner ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function a_star(start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, w::Int64, h::Int64, 
                obstacles::Array{Tuple{Int64, Int64}}, m::a_star_metrics)

    available = PriorityQueue{Tuple{Int64, Int64}, Float64}() # priority queue of states ↦ [ key::(Int64, Int64), value::Float64 ]
    enqueue!(available, start => 0.0)
    g = Dict{Tuple{Int64, Int64}, Float64}(start => 0.0) # dictionary of g from start ↦ { (x::Int64, y::Int64) => cost::Int64) }
    parents = Dict{Tuple{Int64, Int64}, Tuple{Int64, Int64}}(start => (-1, -1)) # dictionary of parent states ↦ { (child_x::Int64, child_y::Int64) => (parent_x::Int64, parent_y::Int64)}
    found_goal = false # whether or not a path is possible

    while !isempty(available)

        current = dequeue!(available) # get best candidate

        if current == goal
            found_goal = true
            break
        end

        neighbors = expand(current, w, h, m) # expand a node (graph operation)

        cost_to_current = g[current]
        for n ∈ neighbors
            edge_cost = evaluate(current, n, w, h, obstacles, m)
            in_queue = n ∈ keys(g)
            if edge_cost < Inf && (!in_queue || cost_to_current + edge_cost < g[n]) # replaces closed set (haven't seen || have seen but cheaper now)
                g[n] = cost_to_current + edge_cost
                f = g[n] + h_manhattan(n, goal)
                parents[n] = current
                in_queue ? available[n] = f : enqueue!(available, n => f)
            end
        end
    end
    return g, parents, m, found_goal
end

function evaluate(state1::Tuple{Int64, Int64}, state2::Tuple{Int64, Int64}, w::Int64, h::Int64, obstacles::Array{Tuple{Int64, Int64}}, m::a_star_metrics)::Float64
    m.evaluations += 1
    if !(0 < state2[1] <= w) || !(0 < state2[2] <= h) || state2 ∈ obstacles
        return Inf
    end
    return √(abs(state2[1]-state1[1])^2 + abs(state2[2]-state1[2])^2)
end

function expand(state::Tuple{Int64, Int64}, w::Int64, h::Int64, m::a_star_metrics)::Array{Tuple{Int64, Int64}}
    m.expansions += 1
    neighbors = Tuple{Int64, Int64}[]
    for a ∈ [(1, 0), (-1, 0), (0, 1), (0, -1) , (-1, -1), (1, 1), (-1, 1), (1, -1)]
        n = (state[1]+a[1], state[2]+a[2]) # neighboring state
        if 0 <= n[1] <= w && 0 <= n[2] <= h
            push!(neighbors, n)
        end
    end
    return neighbors
end

function read_path(path::Array{Tuple{Int64, Int64}}, goal::Tuple{Int64, Int64}, parents::Dict{Tuple{Int64, Int64}, Tuple{Int64, Int64}}, m::a_star_metrics)::Array{Tuple{Int64, Int64}}
    if goal != (-1, -1)
        pushfirst!(path, goal)
        read_path(path, parents[goal], parents, m)
    else
        println("Node Expansions: ", m.expansions)
        println("Full Edge Evaluations: ", m.evaluations)
        return path
    end
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Simulation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function run_sim(start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64},
                w::Int64, h::Int64, object_defs::Dict{String, Any})
    
    terminate = false
    timestep = 1

    # Initialize objects
    object_lists = gen_object_lists(object_defs)
    occupied = flatten_objects(object_lists)
    m = a_star_metrics(0, 0)

    while (terminate == false && timestep <= timelimit) # move objects and robot at each episode, plan for 2 episodes

        # PLAN (only plan during certain episodes)
        @timeit to "Plan" g, parents, m, found_goal = a_star(start, goal, w, h, occupied, m)
 
        # PLOT
        println("STEP: " * string(timestep))
        if found_goal
            println("Path found!")
            @timeit to "Read Path" path = read_path(Tuple{Int64, Int64}[], goal, parents, m) # array of tuples
            plan = path # update buffer
            draw(path, start, goal, w, h, occupied, "./frames/step$(timestep).png")
        else
            println("No path found!")
            draw(Tuple{Int64, Int64}[], start, goal, w, h, occupied, "./frames/environment.png")
        end

        # MOVE OBJECTS
        object_defs = move_objects(w, h, object_defs)
        object_lists = gen_object_lists(object_defs)
        occupied = flatten_objects(object_lists)

        # MOVE ROBOT
        if length(plan) >= robot_speed
            start = plan[robot_speed]
        else
            start = goal
        end

        # TERMINATION CONDITION
        if start == goal
            terminate = true
        end

        timestep += 1

        show(to)
        println("\n")

    end
    return
end

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MAIN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

clean_frames()
run_sim(start, goal, w, h, object_defs)
run(`ffmpeg -framerate 24 -i ./frames/step%d.png -pix_fmt yuv420p videos/a_star_output.mp4`)





