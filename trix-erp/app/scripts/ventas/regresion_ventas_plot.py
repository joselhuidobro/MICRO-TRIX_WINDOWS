from scipy.stats import linregress
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objs as go
import plotly.io as pio

def plot_trend(df_ventas_all, df_week):
    df_ventas_all['Fecha'] = df_ventas_all['Fecha'].str.strip()
    df_ventas_all['Fecha'] = pd.to_datetime(df_ventas_all['Fecha'], format='%Y-%m-%d')

    df_total_fecha = df_ventas_all.groupby('Fecha').agg({'Total_pagado': 'sum'}).reset_index()
    df_total_fecha = df_total_fecha.sort_values(by=['Fecha'])

    df_total_fecha['Año_Semana'] = df_total_fecha['Fecha'].dt.strftime('%Y, semana %V')
    df_suma_semana = df_total_fecha.groupby('Año_Semana').agg({'Total_pagado': 'sum'}).reset_index()
    df_suma_semana['Semana_Num'] = range(1, len(df_suma_semana) + 1)

    slope, intercept, _, _, _ = linregress(df_suma_semana['Semana_Num'], df_suma_semana['Total_pagado'])

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=df_suma_semana['Año_Semana'], y=df_suma_semana['Total_pagado'],
                             mode='markers+lines', marker=dict(size=8), name='Total pagado por semana'))

    fig.add_trace(go.Scatter(x=df_suma_semana['Año_Semana'], y=intercept + slope * df_suma_semana['Semana_Num'],
                             mode='lines', line=dict(color='red'), name='Línea de regresión lineal'))

    # Añadir el total de la semana actual como un punto en la gráfica
    total_semana_actual = df_week['Total_pagado'].sum()
    current_week_str = pd.to_datetime('now').strftime('%Y, semana %V')

    fig.add_trace(go.Scatter(x=[current_week_str],
                             y=[total_semana_actual],
                             mode='markers', marker=dict(color='blue', size=12),
                             name='Total actual semana'))

    # Predicción para la próxima semana
    next_week_num = df_suma_semana['Semana_Num'].iloc[-1] + 1
    predicted_total = intercept + slope * next_week_num
    fig.add_trace(go.Scatter(x=[current_week_str],
                             y=[predicted_total],
                             mode='markers', marker=dict(color='green', size=12),
                             name='Predicción próxima semana'))

    fig.update_layout(title='Tendencia de Ventas por Semana',
                      xaxis_title='Fecha (Año, Semana)',
                      yaxis_title='Total Cobrado',
                      showlegend=True,
                      template='plotly_white',
                      xaxis=dict(tickangle=-45))

    plot_html = pio.to_html(fig, full_html=False)
    return plot_html
