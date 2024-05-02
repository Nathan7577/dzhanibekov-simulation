from sympy import *

def skew(a):
    return Matrix([
        [0, -a[2], a[1]],
        [a[2], 0, -a[0]],
        [-a[1], a[0], 0]])

def unskew(a):
    return Matrix([
        [a[2,1]],
        [a[0,2]],
        [a[1,0]]])

def rot1(t):
    return Matrix([
        [1, 0, 0],
        [0, cos(t), -sin(t)],
        [0, sin(t), cos(t)]])

def rot2(t):
    return Matrix([
        [cos(t), 0, sin(t)],
        [0, 1, 0],
        [-sin(t), 0, cos(t)]])

def rot3(t):
    return Matrix([
        [cos(t), -sin(t), 0],
        [sin(t), cos(t), 0],
        [0, 0, 1]])

latex_dict = {
    r'\sin': r'\textrm{s}',
    r'\cos': r'\textrm{c}',
    r'\tan': r'\textrm{t}',
    r'{\left(t \right)}': '',
    r'\frac{d}{d t}': r'\dot',
    r'\frac{d^{2}}{d t^{2}}': r'\ddot'
}

def replace_values_in_string(text, args_dict=latex_dict):
    for key in args_dict.keys():
        text = text.replace(key, str(args_dict[key]))
    return text

