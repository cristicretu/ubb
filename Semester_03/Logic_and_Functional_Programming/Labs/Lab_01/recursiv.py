class Nod:
    def __init__(self, e):
        self.e = e
        self.urm = None
    
class Lista:
    def __init__(self):
        self.prim = None
        
def creareLista():
    lista = Lista()
    lista.prim = creareLista_rec()
    return lista

def creareLista_rec():
    x = int(input("x="))
    if x == 0:
        return None
    else:
        nod = Nod(x)
        nod.urm = creareLista_rec()
        return nod
    
def tipar(lista):
    tipar_rec(lista.prim)
    
def tipar_rec(nod):
    if nod != None:
        print(nod.e)
        tipar_rec(nod.urm)

# -----------------------------------------------------------
        
# f(l1l2...ln, i, v) = {
#                       [], if n = 0}
#                       v U f(l2...ln, i, v), if i = 1
#                       l1 U f(l2...ln, i-1, v), otherwise
#                       }
def substitute_rec(nod, i, v):
    if nod is None:
        return None
    
    if i == 1:
        nod.e = v
    else:
        substitute_rec(nod.urm, i - 1, v)
    return nod

# a. Substitute the i-th element from a list, with a value v.
def substitute(lista, i, v):
    new_list = Lista()
    new_list.prim = copy_list(lista.prim)
    new_list.prim = substitute_rec(new_list.prim, i, v)
    return new_list

# -----------------------------------------------------------

# notIn(x, l1l2...ln) = {
#                       True, if n = 0
#                       False, if x = l1
#                       notIn(x, l2...ln), otherwise
#                     }
def notIn(node, lista):
    if lista is None:
        return True
    if node.e == lista.e:
        return False
    return notIn(node, lista.urm)
    
# B.
# f(l1l2...ln, k1k2...km) = {
#                           [], if n = 0
#                          l≠ U f(l2...ln, k1k2...lm), if notIn(l1, k1k2...km)
#                          f(l2...ln, k1k2...km), otherwise
#                         }
def difference_rec(nod, list2):
    if nod is None:
        return None
    if notIn(nod, list2):
        new_node = Nod(nod.e)
        new_node.urm = difference_rec(nod.urm, list2)
        return new_node
    else:
        return difference_rec(nod.urm, list2)

# b. Determine difference of two sets represented as lists.
def difference(list1, list2):
    new_list = Lista()
    list1_copy = copy_list(list1.prim)
    list2_copy = copy_list(list2.prim)
    new_list.prim = difference_rec(list1_copy, list2_copy)
    return new_list



def copy_list(nod):
    if nod is None:
        return None
    new_node = Nod(nod.e)
    new_node.urm = copy_list(nod.urm)
    return new_node

def main():
    lista = creareLista()
    print('Initial list')
    tipar(lista)
    result = substitute(lista, 2, 400)
    print('After substituting second element with 400')
    tipar(result)
    list2 = creareLista()
    print('Difference between two lists')
    diff_result = difference(lista, list2)
    tipar(diff_result)
    
main()
