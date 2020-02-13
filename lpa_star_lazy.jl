
using DataStructures
using TimerOutputs
using StatsBase
include("utils.jl")
include("plot.jl")
include("maps.jl")

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

w = 80
h = 80
start = (1, 1)
goal = (70, 70)
robot_speed = 2 # minimum of 2
object_defs = Dict{String, Any}( # center_coord, speed, direction (0=left, 1=up, 2=right, 3=down), radius
    "square_1" => [(50, 50), 5, 2, 1],
    "plus_1" => [(14, 5), 1, 1, 2]
    )
lazy_toggle = true
plan = Array{Tuple{Int64, Int64}}[]
timelimit = 200
to = TimerOutput() # timers


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LPA* Planner ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function lpa_star(Q::PriorityQueue, g::Dict{Tuple{Int64, Int64}, Float64}, rhs::Dict{Tuple{Int64, Int64}, Float64}, 
                    lazy::Dict{Tuple{Int64, Int64}, Bool}, lazy_toggle::Bool,
                    start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, w::Int64, h::Int64, 
                    obstacles::Array{Tuple{Int64, Int64}}, m::lpa_star_metrics)::Tuple{Dict{Tuple{Int64, Int64}, Float64}, Dict{Tuple{Int64, Int64}, Float64}}
    
    while !isempty(Q) && (peek(Q)[2] < get_key(goal, goal, g, rhs) || rhs[goal] != g[goal])

        # → Pop best candidate
        current = dequeue!(Q)
        neighbors = expand(current, w, h, m)
        pushfirst!(m.popped, current)

        # → Lazy evaluations
        if lazy_toggle && lazy[current]
            lazy[current] = false
            for n in neighbors
                rhs[current] = min(rhs[current], g[n] + evaluate(n, current, w, h, obstacles, true, m))
            end
        end

        # → Update g values
        if g[current] > rhs[current] # occupied ↦ free
            g[current] = rhs[current]
        else # free ↦ occupied
            g[current] = Inf
            Q, g, rhs = update_state(current, Q, g, rhs, lazy, false, start, goal, w, h, obstacles, m)
        end

        # → Update neighbors
        for n in neighbors
            Q, g, rhs = update_state(n, Q, g, rhs, lazy, false, start, goal, w, h, obstacles, m)
        end

    end
    return g, rhs
end

function update_state(state::Tuple{Int64, Int64}, Q::PriorityQueue, 
                g::Dict{Tuple{Int64, Int64}, Float64}, rhs::Dict{Tuple{Int64, Int64}, Float64}, 
                lazy::Dict{Tuple{Int64, Int64}, Bool}, lazy_toggle::Bool,
                start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, w::Int64, h::Int64, 
                obstacles::Array{Tuple{Int64, Int64}}, m::lpa_star_metrics)::Tuple{PriorityQueue, Dict{Tuple{Int64, Int64}, Float64}, Dict{Tuple{Int64, Int64}, Float64}}
    
    if state != start

        rhs[state] = Inf

        # Get the rhs value for this state
        neighbors = expand(state, w, h, m)
        lazy_toggle ? lazy[state] = true : lazy[state] = false
        for n in neighbors
            rhs[state] = min(rhs[state], g[n] + evaluate(n, state, w, h, obstacles, lazy_toggle, m))
        end

        # Remove from queue → if locally inconsistent, then it will be added back
        if state ∈ keys(Q)
            delete!(Q, state)
        end

        # Check for local consistency, and add to queue accordingly
        if g[state] != rhs[state]
            enqueue!(Q, state => get_key(state, goal, g, rhs))
        end

    end
    return Q, g, rhs
end

function expand(state::Tuple{Int64, Int64}, w::Int64, h::Int64, m::lpa_star_metrics)::Array{Tuple{Int64, Int64}}
    m.tally ? m.expansions += 1 : nothing
    neighbors = Tuple{Int64, Int64}[]
    for a ∈ [(1, 0), (-1, 0), (0, 1), (0, -1), (-1, -1), (1, 1), (-1, 1), (1, -1)]
        n = (state[1]+a[1], state[2]+a[2]) # neighboring state
        if 0 <= n[1] <= w && 0 <= n[2] <= h
            push!(neighbors, n)
        end
    end
    return neighbors
end

function evaluate(state1::Tuple{Int64, Int64}, state2::Tuple{Int64, Int64}, 
                w::Int64, h::Int64, obstacles::Array{Tuple{Int64, Int64}}, 
                lazy_toggle::Bool, m::lpa_star_metrics)::Float64
    m.tally ? m.evaluations += 1 : nothing

    # → compute lazy evaluation
    if lazy_toggle
        m.tally ? push!(m.eval_lazy, (state1, state2)) : nothing
        return 1
    else
        m.tally ? push!(m.eval_full, (state1, state2)) : nothing
    end

    # → check obstacles and boundaries
    if state2 ∈ obstacles || !((0 < state2[1] <= w) && (0 < state2[2] <= h))
        return Inf
    end

    # → manhattan distance
    return h_manhattan(state1, state2)
end

function get_key(state::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, 
                g::Dict{Tuple{Int64, Int64}, Float64}, rhs::Dict{Tuple{Int64, Int64}, Float64})::Array{Float64}
    return [min(g[state], rhs[state]) + h_manhattan(state, goal), min(g[state], rhs[state])]
end

function read_path(path::Array{Tuple{Int64, Int64}}, g::Dict{Tuple{Int64, Int64}, Float64}, 
                start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, w::Int64, h::Int64, 
                obstacles::Array{Tuple{Int64, Int64}}, m::lpa_star_metrics)::Tuple{Bool, Array{Tuple{Int64, Int64}}}

    pushfirst!(path, goal)

    if goal == start
        return true, path
    end

    if g[goal] == Inf
        return false, path
    else
        p = best_parent(goal, g, w, h, obstacles, m)
        if p == (-1, -1)
            return false, path
        end
        found_path, path = read_path(path, g, start, p, w, h, obstacles, m)
    end 

    return found_path, path
end

function best_parent(state::Tuple{Int64, Int64}, g::Dict{Tuple{Int64, Int64}}, 
                w::Int64, h::Int64, obstacles::Array{Tuple{Int64, Int64}}, m::lpa_star_metrics)::Tuple{Int64, Int64}
    best_cost = Inf
    best = (-1, -1)
    neighbors = expand(state, w, h, m)
    for n in neighbors
        cost = g[n] + evaluate(n, state, w, h, obstacles, false, m)
        if cost < best_cost
            best_cost = cost
            best = n
        end
    end
    return best
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Simulation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function run_sim(start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, 
                w::Int64, h::Int64, object_defs::Dict{String, Any}, timelimit::Int64, lazy_toggle::Bool)

    # INITIALIZATION
    m = lpa_star_metrics(true, 0, 0, Array{Int64}[], Array{Int64}[], Array{Int64}[], Array{Int64}[],
        Array{Tuple{Int64, Int64}}[], Array{Tuple{Int64, Int64}}[],
        Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}}[],
        Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}}[])
    terminate = false
    timestep = 1
    plan = Array{Tuple{Int64, Int64}}[]

    # → Initialize obstacles
    object_lists = gen_object_lists(object_defs)
    occupied = flatten_objects(object_lists)

    # → Initialize priority queue
    Q = PriorityQueue()
    enqueue!(Q, start => [h_manhattan(start, goal), 0.0]) # keys: (x, y), values: [min(g, rhs) + h, min(g, rhs)]

    # → Initialize g table, rhs table
    g = Dict{Tuple{Int64, Int64}, Float64}()
    rhs = Dict{Tuple{Int64, Int64}, Float64}()
    lazy = Dict{Tuple{Int64, Int64}, Bool}()
    grid = [(r, c) for r in -1:h+2, c in -1:w+2]
    for s in grid
        s == start ? (g[s]=Inf ; rhs[s]=0.0 ; lazy[s]=true) : (g[s]=Inf ; rhs[s]=Inf ; lazy[s]=true)
    end

    # MAIN LOOP
    while (terminate == false && timestep <= timelimit) # move objects and robot at each episode, plan for 2 episodes

        # → get shortest path
        @timeit to "Compute Path" g, rhs = lpa_star(Q, g, rhs, lazy, lazy_toggle, start, goal, w, h, occupied, m) # -> need to read it off and display it

        # → read path, and draw it
        m.tally = false
        @timeit to "Read Path" found_path, path = read_path(Tuple{Int64, Int64}[], g, start, goal, w, h, occupied, m)
        if found_path
            println("$(timestep) --> FINISHED PLANNING!")
            plan = path
            draw_lazy(path, start, goal, w, h, occupied, "./frames/step$(timestep).png", m.rhs_updated, m.popped, m.eval_full, m.eval_lazy)
        else
            println("$(timestep) --> NO PATH FOUND!")
            draw_lazy(Tuple{Int64, Int64}[], start, goal, w, h, occupied, "./frames/step$(timestep).png", m.rhs_updated, m.popped, m.eval_full, m.eval_lazy)
        end
        println("Node Expansions: $(m.expansions)")
        println("Edge Evaluations: $(m.evaluations)")
        println("Full Evalutaions: $(length(m.eval_full))")
        println("Lazy Evalutaions: $(length(m.eval_lazy))")

        # → update metrics
        push!(m.all_evaluations, m.evaluations)
        push!(m.all_expansions, m.expansions)
        push!(m.all_full_evals, length(m.eval_full))
        push!(m.all_lazy_evals, length(m.eval_lazy))
        m.tally = true
        m.expansions = 0
        m.evaluations = 0 
        m.rhs_updated = Array{Tuple{Int64, Int64}}[]
        m.popped = Array{Tuple{Int64, Int64}}[]
        m.eval_full = Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}}[]
        m.eval_lazy = Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}}[]

        # → change environment
        object_defs = move_objects(w, h, object_defs)
        object_lists = gen_object_lists(object_defs)
        new_occupied = flatten_objects(object_lists)
        obstacle_changes = [filter(x -> x ∉ occupied, new_occupied) ; filter(x -> x ∉ new_occupied, occupied)]
        occupied = new_occupied

        # → compute new rhs values
        for s in obstacle_changes
            @timeit to "Update State" Q, g, rhs = update_state(s, Q, g, rhs, lazy, lazy_toggle, start, goal, w, h, occupied, m)
            push!(m.rhs_updated, s)
        end

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

        # → display benchmarks
        show(to)
        println("\n")

    end

    # → display overall stats
    println("OVERALL STATS: ")
    println("-"^70)
    exp_mean, exp_std = mean_and_std(m.all_expansions)
    eval_mean, eval_std = mean_and_std(m.all_evaluations)
    full_mean, full_std = mean_and_std(m.all_full_evals)
    lazy_mean, lazy_std = mean_and_std(m.all_lazy_evals)
    println("Expansions (μ, σ): ($(exp_mean), $(exp_std))")
    println("Total Evaluations (μ, σ): ($(eval_mean), $(eval_std))")
    println("Full Evaluations (μ, σ): ($(full_mean), $(full_std))")
    println("Lazy Evaluations (μ, σ): ($(lazy_mean), $(lazy_std))")
    println("-"^70)
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MAIN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

clean_frames()
run_sim(start, goal, w, h, object_defs, timelimit, lazy_toggle)
run(`ffmpeg -framerate 24 -i ./frames/step%d.png -pix_fmt yuv420p lpa_star_output.mp4`)







