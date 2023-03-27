import re
import os
from urllib import parse

# 匹配数学块，DOTALL匹配多行
multi_line_formula_pattern = re.compile(r'\$\$(?P<value>.*?)\$\$', re.DOTALL)
# 匹配行内公式
inline_formula_pattern = re.compile(r'\$(?P<value>.*?)\$', re.S)


# 数学块转换
def MtoImage(matched):
    encoded = parse.quote(matched.group('value'))
    return '\n![](https://www.zhihu.com/equation?tex=' + encoded + ')\n'


# 行内公式转换
def StoImage(matched):
    encoded = parse.quote(matched.group('value'))
    return '![](https://www.zhihu.com/equation?tex=' + encoded + ')'

need_commit="false"
for root,dirs,files in os.walk("./"):
    for file_name in files:
        if file_name.endswith(".md"):
            with open(os.path.join(root,file_name),"a+", encoding='utf-8') as md_file:
                md_file.seek(0)
                text = md_file.read()
                text2 = re.sub(multi_line_formula_pattern, MtoImage, text)
                text3 = re.sub(inline_formula_pattern, StoImage, text2)
                if text != text3:
                    need_commit="true"
                    md_file.truncate(0)
                    md_file.write(text3)
print(need_commit)
