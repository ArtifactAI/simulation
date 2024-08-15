import pandas as pd
from scipy.interpolate import RegularGridInterpolator


class Interpolator:
    def __init__(self, n_independent_vars, data_file=None, dataframe=None):
        if data_file is not None:
            self.df = pd.read_csv(data_file)
        elif dataframe is not None:
            self.df = dataframe
        self.n_independent_vars = n_independent_vars
        self.independent_vars = self.df.iloc[:, :self.n_independent_vars].values
        self.dependent_vars = self.df.iloc[:, self.n_independent_vars:].values
        self.grid_axes = [sorted(set(self.independent_vars[:, i])) for i in range(self.n_independent_vars)]
        self.interpolators = [
            RegularGridInterpolator(self.grid_axes, self.dependent_vars[:, i].reshape([len(ax) for ax in self.grid_axes]))
            for i in range(self.dependent_vars.shape[1])
        ]

    def interpolate(self, *args, clamp=True):
        if not isinstance(args, list):
            args = [args]
        
        if clamp:
            # Clamp the input args to the grid boundaries
            lookup_args = []
            for i, arg in enumerate(args):
                min_val = min(self.grid_axes[i])
                max_val = max(self.grid_axes[i])
                lookup_args.append(max(min(arg, max_val), min_val))
        else:
            lookup_args = args
        
        return [interp(lookup_args).item() for interp in self.interpolators]