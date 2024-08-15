def discrete_gust(V_wind_NED, T_s):
    """
    Generate a discrete gust in the NED frame
    """
    V_wind_NED = np.array(V_wind_NED)
    T_s = np.array(T_s)