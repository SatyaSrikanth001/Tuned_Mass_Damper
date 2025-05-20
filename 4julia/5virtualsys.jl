using GLMakie
using DifferentialEquations
using ParameterizedFunctions
using Formatting

fig = Figure(resolution=(800, 600))
ax = Axis(fig[1, 1], aspect=1)

# Simplified SliderGrid for a single mass-spring-damper system
lsgrid = SliderGrid(fig[1,2],
    (label = "Initial pos", range = -1:0.01:1, startvalue = 0, format = "{:.2f}m"),
    (label = "Initial vel", range = -1:0.01:10, startvalue = 0, format = "{:.2f}m/s"),
    (label = "Stiffness", range = 0:0.01:10, startvalue = 1, format = "{:.2f}Ns/m"),
    (label = "Damping", range = 0:0.01:10, startvalue = 0, format = "{:.2f}N/m"),
    (label = "Mass", range = 0.1:0.01:10, startvalue = 1, format = "{:.2f}kg")
)

# Extract slider values
sliderobservables = [s.value for s in lsgrid.sliders]
x₀, v₀, k, c, m = sliderobservables

# Define ODE system for single mass-spring-damper
odeSystem = @ode_def begin
    dx = v
    dv = (-k * x - c * v) / m
end k c m

tspan = (0, 150)

# Solving the ODE and plotting
points = @lift begin
    u₀ = [$x₀, $v₀]  # Initial conditions
    p = [$k, $c, $m]  # Parameters
    prob = ODEProblem(odeSystem, u₀, tspan, p, reltol=1e-14, abstol=1e-14)
    sol = solve(prob, alg=Vern9())
    y = [x[1] for x in sol.u]  # Position over time
    t = sol.t
    Point2f.(t, y)
end

lines!(points, linewidth=3, color=:blue)

display(fig)
