import re,sys
def parse(s):
    toks=re.findall(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+',s)
    st=[[]]
    for t in toks:
        if t=='(':st.append([])
        elif t==')':x=st.pop();st[-1].append(x)
        else:st[-1].append(t.strip('"') if t.startswith('"') else t)
    return st[0][0]
def get(l,k):
    for x in l:
        if isinstance(x,list) and x and x[0]==k: return x
def load(fn):
    t=parse(open(fn).read())
    comps={}
    for c in get(t,'components')[1:]:
        ref=get(c,'ref')[1]; val=get(c,'value')[1]
        f={}
        fl=get(c,'fields')
        if fl:
            for x in fl[1:]:
                n=get(x,'name')
                f[n[1]]=x[2] if len(x)>2 else ''
        comps[ref]=(val,f)
    nets=[]
    for n in get(t,'nets')[1:]:
        name=get(n,'name')[1]
        nodes=[(get(x,'ref')[1],get(x,'pin')[1]) for x in n if isinstance(x,list) and x[0]=='node']
        nets.append((name,nodes))
    return comps,nets
if __name__=='__main__':
    comps,nets=load(sys.argv[1])
    focus=sys.argv[2].split(',')
    seen=set()
    for name,nodes in nets:
        if any(r in focus for r,p in nodes) and 'GND' not in name.upper():
            print(name,':',' '.join(f'{r}.{p}' for r,p in nodes))
            seen|={r for r,p in nodes}
    print()
    for r in sorted(seen,key=lambda x:(re.sub(r'\d','',x),int(re.sub(r'\D','',x) or 0))):
        v,f=comps[r]; print(r,v,f.get('MPN',''),f.get('Manufacturer',''),f.get('Note',''))
