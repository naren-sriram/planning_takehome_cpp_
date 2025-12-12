# Solution: Safety Analysis and Daily Order Capacity

## How the Planner Works

The path planner uses a **Constrained Shortest Path Problem (CSPP)** approach to find optimal drone delivery routes. The algorithm operates in three phases:

1. **Outbound Search**: Starting from the origin dock, the planner searches for paths to the delivery location. It explores all possible routes while tracking both the number of steps taken and the accumulated population density (risk) along each path.

2. **Inbound Search**: After delivery, the planner searches from the delivery location to find paths to any available dock. Similar to the outbound phase, it tracks steps and density for all possible routes.

3. **Profile Merging**: The planner combines the results from both phases to find the optimal split between outbound and inbound steps. It selects the combination that minimizes total population density while ensuring the combined path length stays within the 110-step budget.

The algorithm uses an augmented state space where each state represents `(position, steps_taken, accumulated_density)`. This allows the planner to simultaneously optimize for both path length (constrained to ≤110 steps) and population density (minimized). The search uses A*-like heuristics to efficiently explore the space while guaranteeing optimality within the step constraint.

### Optimality Guarantees

The CSPP approach provides **provable optimality** in terms of density minimization subject to the step constraint:

1. **Complete State Space Exploration**: By augmenting the state space to include steps taken, the algorithm explores all feasible paths that satisfy the step constraint. For each position and step count combination, it maintains the minimum density path to that state.

2. **Optimal Substructure**: The problem exhibits optimal substructure—the optimal path to any state `(position, steps)` must consist of optimal subpaths. The algorithm exploits this by storing the minimum density for each `(position, steps)` pair and only updating when a better path is found.

3. **Global Optimization via Profile Merging**: The three-phase approach ensures global optimality across both legs. The outbound and inbound searches generate complete risk profiles (minimum density for each step count), and the merge phase exhaustively evaluates all valid combinations to find the globally optimal split.

4. **No Budget Myopia**: Unlike sequential planning approaches, the CSPP method considers the entire path budget simultaneously. The merge phase evaluates all possible allocations of the 110-step budget between outbound and inbound legs, preventing the "budget myopia" problem where the first leg greedily consumes most of the budget.

### Comparison with Lambda-Based Approach

During development, we explored a weighted A* approach using a lambda parameter to balance density and path length. However, this approach suffered from a **budget myopia problem**:

**The Budget Myopia Problem**: In sequential two-leg planning with a strict step budget, the first leg (origin → delivery) doesn't know how much budget the second leg (delivery → dock) will need. With low lambda values, the first leg aggressively optimizes for density by taking long detours, consuming 80-95 steps of the 110-step budget. This leaves only 15-30 steps for the second leg, forcing it to take high-density shortcuts to reach any dock within the remaining budget. The result is that lower lambda (which should favor density) paradoxically produces worse total density because the second leg is starved of budget.

The CSPP approach solves this by:
- **Joint Optimization**: The merge phase considers all possible budget allocations simultaneously
- **No Sequential Greediness**: Neither leg can "steal" budget from the other; the optimal allocation is determined globally
- **Guaranteed Optimality**: The algorithm finds the true minimum-density path within the step constraint, not just a locally optimal solution

## Safety Analysis: Daily Order Capacity

### Methodology

To determine how many orders can be safely fulfilled per day, we analyze the density distribution from our planner's results and apply the given incident probability model.

**Incident Probability Model**: A flight accumulating `d` population density units has an incident probability of `d × 10⁻¹²`.

### Density Statistics

From analysis of 200 orders processed by the planner:

- **Mean density**: 30.71 units
- **Median density**: 30 units
- **95th percentile**: 50 units
- **99th percentile**: 66 units
- **Maximum density**: 70 units

### Daily Incident Probability Calculation

For `n` orders per day, the daily incident probability is:

```
P(daily incident) = 1 - ∏(1 - p_i)
```

where `p_i = density_i × 10⁻¹²` is the incident probability for order `i`.

For small probabilities, this can be approximated using the union bound:

```
P(daily incident) ≈ ∑p_i = n × E[density] × 10⁻¹²
```

### Daily Capacity Estimates

To determine safe daily capacity, we use the 95th percentile density (50 units) to account for worst-case scenarios and variability. We provide estimates for different safety margins:

#### Estimate with 1× Safety Margin (Target: 1 × 10⁻⁸)

Using the target probability directly:

```
n × 50 × 10⁻¹² ≤ 1 × 10⁻⁸
n ≤ (1 × 10⁻⁸) / (50 × 10⁻¹²)
n ≤ 200 orders/day
```

**Estimate: 200 orders per day**

#### Estimate with 1.5× Safety Margin (Target: 6.67 × 10⁻⁹)

Using a 1.5× safety margin (targeting 2/3 of the threshold):

```
n × 50 × 10⁻¹² ≤ 6.67 × 10⁻⁹
n ≤ (6.67 × 10⁻⁹) / (50 × 10⁻¹²)
n ≤ 133.4 orders/day
```

**Recommended Estimate: 133 orders per day**

### Alternative Conservative Estimates (Maximum Density)

Using the maximum observed density (70 units) provides extremely conservative bounds:

**With 1× Safety Margin**:
```
n × 70 × 10⁻¹² ≤ 1 × 10⁻⁸
n ≤ 142.9 orders/day
```

**With 1.5× Safety Margin**:
```
n × 70 × 10⁻¹² ≤ 6.67 × 10⁻⁹
n ≤ 95.3 orders/day
```

These represent extremely conservative estimates assuming every order has the worst-case density.

### Assumptions and Justifications

1. **Independent Flight Incidents**: We assume incidents on different flights are independent events. This is reasonable as each flight operates independently in space and time.

2. **Stationary Density Distribution**: We assume the density distribution of future orders matches the historical distribution analyzed. This is conservative if order patterns remain similar or improve over time.

3. **Union Bound Approximation**: For small probabilities, we use the union bound `P(∪A_i) ≤ ΣP(A_i)`. This is conservative as it overestimates the true probability when events are not mutually exclusive.

4. **95th Percentile Usage**: Using the 95th percentile rather than the mean accounts for:
   - Variability in order characteristics (delivery locations, dock assignments)
   - Potential outliers and edge cases
   - Day-to-day variations in order patterns

5. **Safety Margin**: We provide estimates for both 1× and 1.5× safety margins:
   - **1× margin**: Uses the target probability directly (1 × 10⁻⁸)
   - **1.5× margin**: Targets 6.67 × 10⁻⁹ to provide a buffer for:
     - Model uncertainty in the incident probability formula
     - Unmodeled factors (weather, traffic, system failures)
     - Future changes in order patterns or map characteristics

6. **No Path Degradation**: We assume the planner continues to find optimal paths. If path quality degrades (e.g., due to map changes), densities would increase, making our estimate conservative.

7. **Single Planner Instance**: The analysis assumes a single planner instance. If multiple planners operate in parallel, the daily capacity scales proportionally (e.g., 2 planners → 200 orders/day).

### Conclusion

**Recommended Daily Capacity: 133 orders per day**

This estimate is based on the 95th percentile density (50 units) with a 1.5× safety margin, targeting a daily incident probability of 6.67 × 10⁻⁹. This provides a comfortable buffer below the `10⁻⁸` target while maintaining operational flexibility.

**Summary of Estimates**:

| Safety Margin | 95th Percentile (50 units) | Maximum Density (70 units) |
|---------------|---------------------------|---------------------------|
| 1× (1 × 10⁻⁸) | 200 orders/day | 143 orders/day |
| 1.5× (6.67 × 10⁻⁹) | **133 orders/day** (recommended) | 95 orders/day |

For mission-critical applications requiring maximum safety, the more conservative estimate of 95 orders/day (using maximum density with 1.5× margin) can be used.

The analysis demonstrates that the planner can safely handle a substantial daily order volume while maintaining the required safety standards. The CSPP approach's optimality guarantees ensure that these estimates are based on the best-possible paths within the step constraint.

