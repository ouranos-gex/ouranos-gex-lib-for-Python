# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
from imghdr import what
import os
from pickle import FALSE
import sys
sys.path.insert(0, os.path.abspath('../../src'))

# -- Project information -----------------------------------------------------

project = '空間ID 共通ライブラリ仕様書'
copyright = '<copyright>'
author = ''

# The full version, including alpha/beta/rc tags
release = '../SpatialId'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ["sphinx.ext.autodoc"]
# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']
source_suffix = '.rst'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = 'ja'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'alabaster'
autodoc_typehints = 'description'  # 型ヒントを有効
#autoclass_content = 'class'  # クラスのdocのみ出力
html_theme = 'sphinx_rtd_theme'
autodoc_class_signature = "separated"
autodoc_default_options = {'private-members': False,  # 非公開を非表示
                           'undoc-members': False,
                           'member-order': 'bysource',
                           'class-doc-from': False,
                           'special-members': False,
                           'show-inheritance': False}  # 継承を表示
autoclass_content = "class"
#autodoc-skip-member('class')
# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = []
latex_docclass = {'manual': 'jsbook'}