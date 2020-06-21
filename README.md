# Lazy LPA*

A summer project implementing [A*](https://en.wikipedia.org/wiki/A*_search_algorithm), [LPA*](https://www.cs.cmu.edu/~maxim/files/aij04.pdf), and a homemade lazy variant of LPA* in Julia. Demo videos included in the 'videos' directory.

## Running it

1) Make sure you have Julia installed: https://julialang.org/

2) Create a 'frames' directory in the main directory. Static images will be saved here, and then combined into a video using ffmpeg.

3) **To run A star:**
julia a_star.jl

4) **To run lazy LPA star:**
julia lpa_star_lazy.jl

5) Videos are saved as a_star.mp4 and lpa_star.mp4 depending on which algorithm you run. 

6) The simulation settings (start, goal, obstacles positions/shapes/speeds, etc) are located at the top of a_star.jl and lpa_star_lazy.jl.

## Cool things to try!

- Implement a dynamic lookahead distance that changes based on how chaotic or unpredictable the environment is
- Incorporate side by side planning/execution/world dynamics using parallel computing: https://docs.julialang.org/en/v1/manual/parallel-computing/
- Add more obstacle types and behaviors!

## Bugs

- Collision with an object currently just throws an error and stops planning.
