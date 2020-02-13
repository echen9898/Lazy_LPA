
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Metrics ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

mutable struct a_star_metrics
    expansions::Int64 # keep track of node expansions
    evaluations::Int64 # keep track of true edge evaluations
end

mutable struct lpa_star_metrics
    tally::Bool # keep track of whether or not to tally
    expansions::Int64 # keep track of node expansions for each frame
    evaluations::Int64 # keep track of total edge evaluations for each frame
    all_expansions::Array{Int64} # list of node expansions for each frame
    all_evaluations::Array{Int64} # list of edge evaluations for each frame
    all_full_evals::Array{Int64} # list of full edge evaluations for each frame
    all_lazy_evals::Array{Int64} # list of lazy edge evaluations for each frame
    rhs_updated::Array{Tuple{Int64, Int64}} # list of tile coordinates whose rhs values are updated
    popped::Array{Tuple{Int64, Int64}} # list of tile coordinates popped during path computation
    eval_full::Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}} # fully evaluated edges
    eval_lazy::Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}} # lazily evaluated edges
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Distances ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function h_manhattan(state::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64})::Float64
    return √(abs(goal[1]-state[1])^2 + abs(goal[2]-state[2])^2) # evaluate edge price
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Misc ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function clean_frames()
    println("Clearing images...")
    img_path = "./frames/"
    for (current_dir, dirs, files) ∈ walkdir(img_path)
        for f in files
            f != img_path * ".DS_Store" ? rm(img_path * f) : continue
        end
    end
end