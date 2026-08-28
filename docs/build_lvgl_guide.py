from pathlib import Path
import markdown

root = Path(r"D:\code\tc264\Seekfree_TC264_Opensource_Library\docs")
md_path = root / "lvgl_beginner_guide.md"
html_path = root / "lvgl_beginner_guide.html"

md = md_path.read_text(encoding="utf-8")
html = markdown.markdown(md, extensions=["fenced_code", "tables"])
page = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<style>
body{font-family:Arial,"Microsoft YaHei",sans-serif;line-height:1.55;max-width:980px;margin:40px auto;padding:0 16px;color:#222}
img{max-width:100%;height:auto}
pre{background:#f6f8fa;padding:12px;overflow:auto;border-radius:6px}
code{font-family:Consolas,monospace}
</style>
</head>
<body>
""" + html + """
</body>
</html>
"""
html_path.write_text(page, encoding="utf-8")
print(html_path)
