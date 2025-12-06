Background
------------

Zipline is considering operating in San Francisco! We anticipate a very large demand for drone
deliveries, so we'd like to serve as many customers as possible while making sure our system is
safe.

Zipline's drones ("zips") are designed to take off from a dock, fly to a delivery site, drop off the
order, and then fly back to any dock. Your goal for this takehome problem is to:

* Write a planner that gets the zip from the source dock to the delivery site, and from the delivery
site to a destination dock
* Show that your planner meets Zipline's safety targets

Part 1
-------

You MUST complete this portion of the coding challenge in C++ or Rust. Please do not use any
external libraries -- only use the standard library.

Zipline expects to receive a larger volume of orders from locations with a higher population
density. To help us model order demand, our GIS engineers have created a map that divides San
Francisco into a 2D grid. Each cell in the grid encodes a value from 0-9 representing population
density, where 0 means there are no customers in the cell and 9 means there are many customers. We
expect the order volume from a given cell to be proportional to this population density. The map
also contains the possible dock sites, which never change.

As with any physical system, there is a risk that something goes wrong mid-flight. Luckily, zips are
equipped with a safety parachute that lowers them gently to the ground. To be extra safe, we'd still
like to minimize the likelihood that a zip will parachute over a densely populated area. So, while
orders are more likely to come from high-density areas, we'd much rather fly over low-density areas.

Here are the requirements for your planner:

1. Find a connected path that completes the order. An order consists of a source dock site and a
delivery site. The path must connect the source dock site to the delivery site along an 8-connected
grid (i.e. lateral, vertical, and diagonal steps). It must then connect the delivery site to any of
the dock sites in the map.
2. The total length of the flight (outbound path + inbound path) must stay below 110 steps. Lateral,
vertical, and diagonal steps count as 1 step each. Your planner must be capable of servicing all
possible orders within San Francisco. For example, the farthest possible order in San Francisco
originates from the dock at (10, 25) to deliver at (63, 10). If we return back at the origin dock,
the total length of the flight will be a total of 2 * max(63-10, 25-10) = 106 steps.
3. The path should attempt to minimize the sum of the population density we fly over given the two
constraints above.

Luckily, we are providing you a solution that already meets the first two requirements! To get
started, run `./test.sh`. The script does a few things:

* Builds the planner. The code is included under the `cpp` directory. You can run `make` inside that
directory to build the `order_planner` binary. You will need Make and a version of GCC that
supports the C++20 standard.
* Generates 5 random orders proportional to population density using the `--generate_random_orders`
flag of `analysis.py`. `analysis.py` requires `matplotlib` and `numpy`.
* Runs the `order_planner` binary to come up with paths for the orders.
* Evaluates and visualizes the paths using the `--plot` flag of `analysis.py`.

Before you start, get acquainted with the codebase. Some notes:

* The 2D density array is indexed by (east, north) -- careful!
* See `utils.cpp` if you'd like to understand the file formats for maps, orders, and paths. It may
be useful to create new map or order files to develop or test your solution.
* A successful submission for Part 1 can happen entirely within `planner.h` and `planner.cpp`, but
feel free to modify other files as long as the `test.sh` harness continues to work.

While the optimality of your solution is important, you will be evaluated on code quality and
correctness first.

Part 2
-------

You may use Python, C++, or Rust for this portion of the coding challenge. You are allowed to use
scipy, numpy, and matplotlib, but please do not use any other libraries beyond the standard
libraries.

Our goal now is to understand how many customers we can serve safely given how well our planner
performs! Specifically, we want to fulfill as many orders as possible per day while keeping the
probability of a safety incident on a given day under 1e-8.

We will make a crude approximation that a flight that accumulates `d` population density "units" has
an incident probability of `d * 1e-12`. For example, if a given flight accumulates 50 density units,
the probability of an incident on that flight is `50e-12`.

The output of this portion of the takehome should be a README (feel free to use a text file or a
PDF). In your README:

* Give a brief explanation of how your planner works.
* Give a concrete estimate of the number of orders per day that your planner can fulfill in San
Francisco while staying under our safety target.
* Explain how you determined the estimate and any intermediate steps along the way.
* State all assumptions that went into the estimate and explain why they are conservative. You are
encouraged to simplify the problem if necessary.

Feel free to improve the planner as you work through this problem, but please prioritize giving a
convincing safety argument! We're excited to read about your approach.
