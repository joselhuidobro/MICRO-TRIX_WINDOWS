project = "TRIX Docs"
extensions = ["myst_parser", "breathe"]
html_theme = "sphinx_rtd_theme"

# Doxygen XML (montado en /docs/_doxygen/xml)
breathe_projects = {"trix": "_doxygen/xml"}
breathe_default_project = "trix"
