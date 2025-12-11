#!/usr/bin/env python3
"""
Density Distribution Visualization
Creates a bell curve/histogram showing density distribution with key statistics.
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import os
import glob
import re

def read_densities_from_csv(csv_file):
    """Read all density values from CSV file (skips header row)."""
    densities = []
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        header = next(reader, None)  # Skip header row
        for row in reader:
            if len(row) >= 6:
                try:
                    density = int(row[5])  # 6th column: total_density (0-indexed: 5)
                    densities.append(density)
                except (ValueError, IndexError):
                    continue
    return densities

def main():
    # Get the script directory (visualizer folder)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, 'results.csv')
    
    # Check if file exists
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found!")
        return
    
    # Read all densities from results.csv
    all_densities = read_densities_from_csv(csv_file)
    
    if not all_densities:
        print("Error: No density values found!")
        return
    
    # Calculate statistics
    mean = np.mean(all_densities)
    median = np.median(all_densities)
    std = np.std(all_densities)
    pct_95 = np.percentile(all_densities, 95)
    pct_99 = np.percentile(all_densities, 99)
    min_dens = np.min(all_densities)
    max_dens = np.max(all_densities)
    
    print(f"Statistics:")
    print(f"  Mean: {mean:.2f}")
    print(f"  Median: {median:.2f}")
    print(f"  Std Dev: {std:.2f}")
    print(f"  95th percentile: {pct_95:.2f}")
    print(f"  99th percentile: {pct_99:.2f}")
    print(f"  Min: {min_dens}")
    print(f"  Max: {max_dens}")
    print(f"  Sample size: {len(all_densities)}")
    print(f"  Data source: {csv_file}")
    print()
    
    if len(all_densities) == 0:
        print("Warning: No density values found in CSV file!")
        return
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Create histogram
    n_bins = 30
    counts, bins, patches = ax.hist(all_densities, bins=n_bins, density=True, 
                                    alpha=0.7, color='steelblue', edgecolor='black', linewidth=1.2)
    
    # Fit normal distribution
    x = np.linspace(min_dens - 10, max_dens + 10, 1000)
    if std > 0:
        normal_curve = stats.norm.pdf(x, mean, std)
        ax.plot(x, normal_curve, 'r-', linewidth=2.5, label=f'Normal Distribution (μ={mean:.1f}, σ={std:.1f})')
    
    # Mark statistics with vertical lines
    # Mean
    ax.axvline(mean, color='green', linestyle='--', linewidth=2.5, label=f'Mean = {mean:.2f}')
    ax.text(mean, ax.get_ylim()[1] * 0.95, f'Mean\n{mean:.1f}', 
            ha='center', va='top', fontsize=11, fontweight='bold', 
            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
    
    # Median
    ax.axvline(median, color='orange', linestyle='--', linewidth=2.5, label=f'Median = {median:.2f}')
    ax.text(median, ax.get_ylim()[1] * 0.85, f'Median\n{median:.1f}', 
            ha='center', va='top', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
    # 95th percentile
    ax.axvline(pct_95, color='purple', linestyle='--', linewidth=2.5, label=f'95th Percentile = {pct_95:.2f}')
    ax.text(pct_95, ax.get_ylim()[1] * 0.75, f'95th %ile\n{pct_95:.1f}', 
            ha='center', va='top', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='plum', alpha=0.7))
    
    # Standard deviations
    if std > 0:
        # ±1σ
        ax.axvline(mean - std, color='blue', linestyle=':', linewidth=2, alpha=0.7, label=f'μ - 1σ = {mean - std:.1f}')
        ax.axvline(mean + std, color='blue', linestyle=':', linewidth=2, alpha=0.7, label=f'μ + 1σ = {mean + std:.1f}')
        ax.text(mean - std, ax.get_ylim()[1] * 0.65, f'μ-1σ\n{mean - std:.1f}', 
                ha='center', va='top', fontsize=9, color='blue', alpha=0.8)
        ax.text(mean + std, ax.get_ylim()[1] * 0.65, f'μ+1σ\n{mean + std:.1f}', 
                ha='center', va='top', fontsize=9, color='blue', alpha=0.8)
        
        # ±2σ
        ax.axvline(mean - 2*std, color='cyan', linestyle=':', linewidth=2, alpha=0.6, label=f'μ - 2σ = {mean - 2*std:.1f}')
        ax.axvline(mean + 2*std, color='cyan', linestyle=':', linewidth=2, alpha=0.6, label=f'μ + 2σ = {mean + 2*std:.1f}')
        ax.text(mean - 2*std, ax.get_ylim()[1] * 0.55, f'μ-2σ\n{mean - 2*std:.1f}', 
                ha='center', va='top', fontsize=9, color='cyan', alpha=0.8)
        ax.text(mean + 2*std, ax.get_ylim()[1] * 0.55, f'μ+2σ\n{mean + 2*std:.1f}', 
                ha='center', va='top', fontsize=9, color='cyan', alpha=0.8)
    
    # Shade regions
    if std > 0:
        # Shade ±1σ region
        x_1sigma = np.linspace(mean - std, mean + std, 100)
        y_1sigma = stats.norm.pdf(x_1sigma, mean, std)
        ax.fill_between(x_1sigma, y_1sigma, alpha=0.2, color='blue', label='±1σ region (68%)')
        
        # Shade ±2σ region
        x_2sigma = np.linspace(mean - 2*std, mean + 2*std, 100)
        y_2sigma = stats.norm.pdf(x_2sigma, mean, std)
        ax.fill_between(x_2sigma, y_2sigma, alpha=0.15, color='cyan', label='±2σ region (95%)')
    
    # Labels and title
    ax.set_xlabel('Population Density (accumulated units)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Probability Density', fontsize=14, fontweight='bold')
    ax.set_title(f'Density Distribution: Bell Curve with Key Statistics (n={len(all_densities)})', 
                 fontsize=16, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='upper right', fontsize=10, framealpha=0.9)
    
    # Add statistics box
    stats_text = f'Sample Size: {len(all_densities)}\n'
    stats_text += f'Mean (μ): {mean:.2f}\n'
    stats_text += f'Median: {median:.2f}\n'
    stats_text += f'Std Dev (σ): {std:.2f}\n'
    stats_text += f'95th %ile: {pct_95:.2f}\n'
    stats_text += f'99th %ile: {pct_99:.2f}\n'
    stats_text += f'Range: [{min_dens}, {max_dens}]'
    
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    
    # Find next available file number
    def get_next_file_number(directory, base_name, extension):
        """Find the next available file number for incremental naming."""
        pattern = os.path.join(directory, f'{base_name}_*.{extension}')
        existing_files = glob.glob(pattern)
        
        max_num = 0
        for file_path in existing_files:
            # Extract number from filename like "density_distribution_5.png"
            match = re.search(rf'{re.escape(base_name)}_(\d+)\.{re.escape(extension)}', 
                            os.path.basename(file_path))
            if match:
                num = int(match.group(1))
                max_num = max(max_num, num)
        
        return max_num + 1
    
    # Get next file number
    file_num = get_next_file_number(script_dir, 'density_distribution', 'png')
    
    # Save figure with incremented number
    output_file = os.path.join(script_dir, f'density_distribution_{file_num}.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nFigure saved to: {output_file}")
    
    # Also save as PDF with same number
    output_pdf = os.path.join(script_dir, f'density_distribution_{file_num}.pdf')
    plt.savefig(output_pdf, bbox_inches='tight')
    print(f"Figure saved to: {output_pdf}")
    
    # Show plot
    plt.show()

if __name__ == '__main__':
    main()

