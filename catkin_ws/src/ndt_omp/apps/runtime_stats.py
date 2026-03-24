import pandas as pd
import matplotlib.pyplot as plt
import numpy as np



def plot_n_save(df, key, save_path):
    sub_df = df[df[df.columns[0]]  == key]
    print(sub_df.shape)
    mean = sub_df[sub_df.columns[1]].mean()
    q = sub_df[sub_df.columns[1]].quantile(np.linspace(.9, 1, 10, 0))
    print('mean of ' +key +' = ', mean)
    print(q)
    plt.figure()
    plt.plot(sub_df[sub_df.columns[1]])
    plt.axhline(mean, color = 'r')
    plt.title(key)
    plt.savefig(save_path+key[:-1]+'.png')
    
def plot_n_save2(df, key, save_path):
    #sub_df = df[df[df.columns[0]]  == key]
    sub_df = df
    print(sub_df.shape)
    mean = sub_df[sub_df.columns[3]].mean()
    q = sub_df[sub_df.columns[3]].quantile(np.linspace(.1, 1, 10, 0))
    q1 = sub_df[sub_df.columns[3]].quantile(np.linspace(.9, 1, 10, 0))
    print('mean of ' +key +' = ', mean)
    print(q)
    print(q1)
    plt.figure()
    plt.plot(sub_df[sub_df.columns[3]])
    plt.axhline(mean, color = 'r')
    plt.title(key)
    plt.savefig(save_path+key[:-1]+'.png')



save_path ='/workspace/log_diff'
#df= pd.read_csv("/dataset/logs/infra_log.txt", sep = ":",header=None)
df1= pd.read_csv("/workspace/log_time_network_compress.csv", sep = ",")
df2= pd.read_csv("/workspace/log_time_network_diff.csv", sep = ",")
df3= pd.read_csv("/workspace/log_time_network_raw.csv", sep = ",")

print(df1.shape)

#list_of_keys  = ['duration_local_variables ','duration_pc2_to_pc ', 'duration_decompression ', 'duration_reconstruction ', 'duration_inital_guess ', 'duration_downsampling ', 'duration_ICP ', 'duration_transform_infra ', 'duration_fusion ', 'duration_total ']
#list_of_keys = ['Latency_diff ', 'Latency_compress ']
#list_of_keys = ['Latency ', 'Fitness ']


plot_n_save2(df1,"diffrence (ms)",'/workspace/log_compress')
plot_n_save2(df2,"diffrence (ms)",'/workspace/log_diff')
plot_n_save2(df3,"diffrence (ms)",'/workspace/log_raw')

#for i in list_of_keys:
#    plot_n_save(df,i,save_path)


