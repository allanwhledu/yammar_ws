#!/usr/bin/env python
# coding=utf-8
# 测试python的全局变量，局部变量的机制

s_global = 'global variable s_global'
t_global = 'global variable t_global'


def formal_print(s_global):  # 常规的传参用法，传递参数进行print，变量名可任意
    print "formal_print: ", s_global
    return


def global_print():  # 无参数传递，直接对global variable进行print
    print "global_print: ", s_global
    return


def global_print_para(st):  # 此处虽然传递了一个参数st，但是并没有在函数中用到
    print "global_print_para: ", s_global
    return


def test_global():
    stest = 'test_global'
    print "test_global: ", stest
    return


def main():
    # main函数中声明的变量默认为global variable，
    # 而其他def函数中声明的变量则默认为local variable
    s_global = 'main variable s_global'
    t_global = 'main variable t_global'
    formal_print(s_global)
    formal_print(t_global)
    global_print()
    test_global()
    # formal_print(stest)#虽然在test_global()中声明了变量stest，但stest并非全局变量
    print 'End.'


if __name__ == '__main__':
    main()
