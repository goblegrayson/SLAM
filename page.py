"""
page.py
A streamlit page
"""
from pathlib import Path
import numpy as np
import streamlit as st
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import seaborn as sns
from matplotlib.colors import LinearSegmentedColormap
import black_scholes.model as model


# Page Setup
with open(Path(__file__).parent.joinpath('README.md'), 'r', encoding="utf8") as file:
    st.markdown(file.read(), unsafe_allow_html=True)

