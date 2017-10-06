import plotly.plotly as py
import plotly.graph_objs as go
import plotly.tools as FF

import numpy as np
import pandas as pd
df = pd.read_csv('./sensor/CA1393BA_4_8_2017.csv')

sample_data_table = FF.create_table(df.head())
py.iplot(sample_data_table, filename='sample-data-table')
