
using Luxor

white = "#fdfffc"
black = "#02020a"
blue = "#235789"
baby_blue = "#86bbd8"
navy_blue = "#031f40"
dark_green = "#2b761b"
light_green = "#87e752"
red = "#c1292e"
yellow = "#f1d302"
wine = "#61304b"
mint = "#effffa"

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DRAWING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function draw(path::Array{Tuple{Int64, Int64}}, start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, 
                w::Int64, h::Int64, obstacles::Array{Tuple{Int64, Int64}}, file::String)

    resolution = 50 # pixels per path segment
    w_pix = w*resolution
    h_pix = h*resolution
    Drawing(w_pix, h_pix, :png, file)
    background(mint)

    # Draw the board grid
    origin()
    tiles = Tiler(w_pix, h_pix, h, w, margin=0)
    sethue(black)
    for (pos, n) in tiles
        box(pos, tiles.tilewidth, tiles.tileheight, :stroke)
    end

    # Standard bottom left origin
    translate(-w_pix/2, h_pix/2)
    transform([-1 0 0 1 0 0])
    rotate(π)

    # Fill in start/goal tiles
    sethue(red)
    box(create_point(start, resolution), tiles.tilewidth, tiles.tileheight, :fill)
    sethue(dark_green)
    box(create_point(goal, resolution), tiles.tilewidth, tiles.tileheight, :fill)

    # Fill in obstacles
    sethue(black)
    for o ∈ obstacles
        box(create_point(o, resolution), tiles.tilewidth, tiles.tileheight, :fill)
    end

    # Draw the path
    if !isempty(path)
        sethue(red)
        setline(15)
        setlinecap("round")
        setlinejoin("round")
        pixel_path = Point[]
        for p in path
            push!(pixel_path, create_point(p, resolution))
        end
        poly(pixel_path, :stroke, close=false)
    end

    finish()
end


function draw_lazy(path::Array{Tuple{Int64, Int64}}, start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, 
                w::Int64, h::Int64, obstacles::Array{Tuple{Int64, Int64}}, file::String, 
                rhs_updated::Array{Tuple{Int64, Int64}}, explored::Array{Tuple{Int64, Int64}}, 
                eval_fully::Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}},
                eval_lazily::Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}})

    resolution = 50 # pixels per path segment
    w_pix = w*resolution
    h_pix = h*resolution
    Drawing(w_pix, h_pix, :png, file)
    background(mint)

    # Draw the board grid
    origin()
    tiles = Tiler(w_pix, h_pix, h, w, margin=0)
    sethue(black)
    for (pos, n) in tiles
        box(pos, tiles.tilewidth, tiles.tileheight, :stroke)
    end

    # Standard bottom left origin
    translate(-w_pix/2, h_pix/2)
    transform([-1 0 0 1 0 0])
    rotate(π)

    # Fill in start/goal tiles
    sethue(red)
    box(create_point(start, resolution), tiles.tilewidth, tiles.tileheight, :fill)
    sethue(dark_green)
    box(create_point(goal, resolution), tiles.tilewidth, tiles.tileheight, :fill)

    # Fill in obstacles
    sethue(black)
    for o in obstacles
        box(create_point(o, resolution), tiles.tilewidth, tiles.tileheight, :fill)
    end

    # Draw edges evaluated
    !isempty(eval_lazily) ? draw_edges(eval_lazily, baby_blue, 7.0, resolution) : nothing
    !isempty(eval_fully) ? draw_edges(eval_fully, blue, 4.0, resolution) : nothing

    # Draw the path
    if !isempty(path)
        sethue(red)
        setline(15)
        setlinecap("round")
        setlinejoin("round")
        pixel_path = Point[]
        for p in path
            push!(pixel_path, create_point(p, resolution))
        end
        poly(pixel_path, :stroke, close=false)
    end

    # Draw rhs updated and popped nodes
    !isempty(rhs_updated) ? draw_dots(rhs_updated, wine, 6, resolution) : nothing
    !isempty(explored) ? draw_dots(explored, yellow, 11, resolution) : nothing

    finish()
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ HELPER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function draw_dots(coords::Array{Tuple{Int64, Int64}}, color::String, s::Int64, resolution::Int64)
    sethue(color)
    for c in coords
        circle(create_point(c, resolution), resolution/s, :fill)
    end
end

function draw_edges(coords::Array{Tuple{Tuple{Int64, Int64}, Tuple{Int64, Int64}}}, color::String, th::Float64, resolution::Int64)
    sethue(color)
    setline(th)
    setlinecap("round")
    setlinejoin("round")
    for endpts in coords
        p1 = create_point(endpts[1], resolution)
        p2 = create_point(endpts[2], resolution)
        line(p1, p2, :stroke)
    end
end

function create_point(coord::Tuple{Int64, Int64}, resolution::Int64)::Point
    return Point(resolution*(coord[1] - 1/2), resolution*(coord[2] - 1/2))
end


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ANIMATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

function animate(path::Array{Tuple{Int64, Int64}}, w::Int64, h::Int64, obstacles::Array{Tuple{Int64, Int64}})

    resolution = 50 # pixels per path segment
    w_pix = w*resolution
    h_pix = h*resolution
    demo = Movie(w_pix, h_pix, "test")

    function backdrop(scene, framenumber)
        background("gray79")

        # Draw tiles
        origin()
        tiles = Tiler(w_pix, h_pix, h, w, margin=0)
        sethue("black")
        for (pos, n) in tiles
            box(pos, tiles.tilewidth, tiles.tileheight, :stroke)
        end
    end

    function frame(scene, framenumber)

        # Shift origin to bottom left corner
        translate(-w_pix/2, h_pix/2)
        transform([-1 0 0 1 0 0])
        rotate(π)

        # Draw obstacles
        for o in obstacles
            box(create_point(o, resolution), resolution, resolution, :fill)
        end

        # Draw final path
        if !isempty(path)
            sethue("magenta")
            setline(10)
            setlinecap("round")
            setlinejoin("round")
            pixel_path = Point[]
            for p in path
                push!(pixel_path, create_point(p, resolution))
            end
            poly(pixel_path, :stroke, close=false)
        end

        gsave()
        grestore()
    end

    animate(demo, [Scene(demo, backdrop, 0:359), Scene(demo, frame, 0:359)], tempdirectory="./still_frames")
end




















