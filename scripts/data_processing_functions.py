import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as mpl
import seaborn as sns

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

def df_from_csv(csv_path, req_cols):
    return pd.read_csv(csv_path,usecols=req_cols)

def parameter_tuning_read_and_mean(csv_path,tested_parameter,tuning_measure,validation_measures):
    req_cols = ["algName","objName",tested_parameter,tuning_measure]+validation_measures
    df =  pd.read_csv(csv_path,usecols=req_cols,dtype={"algName":"category","obj":"category"})
    return df.groupby(["algName","objName",tested_parameter]).mean().reset_index()

def print_tuned_parameters(df,tuning_measure,tested_parameter):
    max_idx = df.groupby(["algName","objName"])[tuning_measure].idxmax().to_numpy()
    for index, row in df.iloc[max_idx,:].iterrows():
        print("{},{},{},{}".format(row["objName"],row["algName"],tested_parameter,row[tuning_measure]))

def parameter_tuning_validation_barplot(df,tuning_measure,validation_measures,figsize=(20,5)):
    max_idx = df.groupby(["algName","objName"])[tuning_measure].idxmax().to_numpy()

    fig, axes = plt.subplots(1,len(validation_measures),figsize=(20,5),tight_layout=True)

    for key,ax in zip(validation_measures,axes):
        sns.barplot(x="objName", y=key, hue="algName", data=df.loc[max_idx,:],ax=ax)
    return fig,axes

def get_best_cost_corr_parameters(corr_df,corrwith,tuning_measure):
    best_idx = corr_df.groupby(["geName","objName"])[corrwith].idxmin().to_numpy()
    best_params = corr_df.loc[best_idx,["objName","geName",tuning_measure,corrwith]]
    return best_params, best_idx

def print_best_cost_corr_parameters(best_params,tuning_measure):
    for index,row in best_params.iterrows():
        print(row["objName"],row["geName"],tuning_measure,row[tuning_measure],sep=",")