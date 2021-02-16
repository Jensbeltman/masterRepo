import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as mpl

def object_mean_barplot(df,key,ax=None):
    data = df.groupby(["objName","algName"]).mean()
    data = data.pivot_table(index='objName', columns='algName', values=key)
    ax = data.plot.bar(ax=ax)
    ax.set_ylabel(key)
    return ax


def alg_mean(df,key):
    return df.groupby(["algName"])[key].mean()

def alg_mean_barplot(df,key,ax=None):
    alg_mean(df,key)
    data = df
    ax = data.plot.bar(ax=ax)
    ax.set_ylabel(key)
    return ax

def series_alg_mean(data_info):
    mean_frames = []
    for fn in data_info["data_path"]:
        df = pd.read_csv(fn,sep=',')
        mean_frames.append(df.groupby(["algName"]).mean())
    return pd.concat(mean_frames)

def series_alg_mean_line_plot(data_info,keys,xkey,ax):
    df = series_alg_mean(data_info)
    for key in keys:
        ax.plot(data_info[xkey],df[key])
    
    ax.legend(keys)
    ax.set_xlabel(xkey)

    return ax

def series_mean(data_info,sortby, groupby = ["objName","algName"]):
    mean_frames = []
    for index, row in  data_info.iterrows():
        df = pd.read_csv(row["data_path"],sep=',')
        df = df.groupby(groupby).mean()
        df[sortby] = [row[sortby] for i in range(df.shape[0])]
        mean_frames.append(df)
    return pd.concat(mean_frames).sort_values(sortby)

def series_obj_mean_line_plot(data_info,key,variable_key,ax,groupby = ["objName","algName"]):
    df = series_mean(data_info,variable_key,groupby)
    gb = df.groupby(groupby)
    for group in gb.groups:
        data = gb.get_group(group)
        ax.plot(data[variable_key],data[key])
    
    ax.legend(gb.groups)
    ax.set_ylabel(key)
    ax.set_xlabel(variable_key)

    return ax