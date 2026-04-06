from __future__ import annotations

project = "2026 Rebuilt"
author = "MECO Robotics"
copyright = "2026, MECO Robotics"

extensions = [
    "myst_parser",
    "sphinx_copybutton",
]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

root_doc = "index"

templates_path = ["_templates"]
exclude_patterns: list[str] = []

html_theme = "furo"
html_title = "2026 Rebuilt Docs"
html_static_path = ["_static"]
html_css_files = ["custom.css"]

html_theme_options = {
    "navigation_with_keys": True,
    "footer_icons": [
        {
            "name": "Furo",
            "url": "https://github.com/pradyunsg/furo",
            "html": """
                <svg stroke="currentColor" fill="currentColor" stroke-width="0" viewBox="0 0 16 16">
                  <path d="M8 0a8 8 0 1 0 0 16A8 8 0 0 0 8 0Zm3.5 4.5v7h-1.2V8.7H5.7v2.8H4.5v-7h1.2v3h4.6v-3h1.2Z"/>
                </svg>
            """,
            "class": "",
        }
    ],
}

myst_enable_extensions = [
    "colon_fence",
]
