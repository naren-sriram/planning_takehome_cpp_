
import matplotlib.pyplot as plt
import numpy as np
import argparse

from dataclasses import dataclass

@dataclass
class Site:
    e: int
    n: int

@dataclass
class Order:
    origin_dock: Site
    delivery_site: Site

    def __str__(self):
        return f"{self.origin_dock.e},{self.origin_dock.n} {self.delivery_site.e},{self.delivery_site.n}"

@dataclass
class Map:
    docks: list[Site]
    density: np.ndarray

    def __init__(self, filename):
        """
        Read the map from the given file. See main.cpp for the file format.
        """
        with open(filename, 'r') as f:
            n_rows, n_cols = (int(x) for x in f.readline().strip().split())

            density = []
            for _ in range(n_rows):
                density.append([int(x) for x in f.readline().strip().split()])
            self.density = np.array(density)

            self.docks = []
            dock_site_strs = f.readline().strip().split()
            for site_str in dock_site_strs:
                e, n = (int(x) for x in site_str.split(','))
                self.docks.append(Site(e, n))

    def sample_random_delivery_site(self):
        """
        Sample a random delivery site from the map with probability proportional to density,
        excluding the docks.
        """
        density_without_docks = self.density.copy()
        density_without_docks[[s.e for s in self.docks], [s.n for s in self.docks]] = 0
        flattened_density = density_without_docks.ravel()
        probabilities = flattened_density / flattened_density.sum()
        sampled_index_1d = np.random.choice(np.arange(flattened_density.size), p=probabilities)
        sampled_index_2d = np.unravel_index(sampled_index_1d, self.density.shape)
        return Site(sampled_index_2d[0], sampled_index_2d[1])
    
    def sample_random_order(self):
        """
        Sample a random order from the map with a random origin dock, random delivery site, and
        random destination dock.
        """
        origin_dock = np.random.choice(self.docks)
        delivery_site = self.sample_random_delivery_site()
        return Order(origin_dock, delivery_site)
    
    def evaluate_accumulated_density(self, path: list[Site]):
        """
        Evaluate the risk of the given path.
        """
        return sum(self.density[s.e, s.n] for s in path)

    def show_plot(self):
        """
        Display the map.
        """
        plt.imshow(self.density.T, cmap='Wistia', origin='lower')
        plt.plot([s.e for s in self.docks], [s.n for s in self.docks], 'go', label='Dock')
        plt.xlabel("East")
        plt.ylabel("North")
        plt.legend()
        plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map', type=str, help='Path to map file')
    parser.add_argument('--generate_random_orders', type=int, help='The number of random orders to generate')
    parser.add_argument('--paths', type=str, help='Path to file containing paths')
    parser.add_argument('--plot', action='store_true', help='Display the map (and paths, if provided)')
    args = parser.parse_args()

    m = Map(args.map)

    if args.generate_random_orders:
        for _ in range(args.generate_random_orders):
            print(m.sample_random_order())

    if args.paths:
        with open(args.paths, 'r') as f:
            for line in f:
                outbound_str, inbound_str = line.strip().split('//')
                outbound_path = [Site(*[int(x) for x in s.split(',')]) for s in outbound_str.split()]
                inbound_path = [Site(*[int(x) for x in s.split(',')]) for s in inbound_str.split()]
                whole_path = outbound_path[:-1] + inbound_path # Remove the duplicate delivery site
                delivery_site = outbound_path[-1]

                if args.plot:
                    plt.figure(figsize=(10,8))
                    plt.plot([s.e for s in outbound_path], [s.n for s in outbound_path], 'b-', label='Outbound path')
                    plt.plot([s.e for s in inbound_path], [s.n for s in inbound_path], 'c-', label='Inbound path')
                    plt.plot([delivery_site.e], [delivery_site.n], 'ro', label='Delivery site')
                    plt.title(f'Length: {len(whole_path)}\n Accumulated density: {m.evaluate_accumulated_density(whole_path)}')
                    m.show_plot()

    elif args.plot:
        m.show_plot()

if __name__ == '__main__':
    main()
