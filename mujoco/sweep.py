import itertools, json, numpy as np
from multiprocessing import Pool
from tune_eval import evaluate

# barrido grueso sobre params de alto impacto
grid = dict(
    fz_scale=[0.8,1.0,1.3,1.7],
    cw_mass=[1.9,2.1,2.3],
    tst=[0.12,0.15,0.20],
    kd=[10,20,35],
)
keys=list(grid); combos=[dict(zip(keys,v)) for v in itertools.product(*grid.values())]
def ev(p):
    try: r=evaluate(p,t_total=6.0); r["params"]=p; return r
    except Exception as e: return {"score":-99,"params":p,"err":str(e)}
if __name__=="__main__":
    print(f"barriendo {len(combos)} combinaciones...")
    with Pool(6) as pool: res=pool.map(ev, combos)
    res=[r for r in res if r]; res.sort(key=lambda r:r.get("score",-99), reverse=True)
    print("=== TOP 8 ===")
    for r in res[:8]:
        print(f"score={r['score']:.2f} hops={r.get('n_hops')} apex_std={r.get('apex_std',9):.3f} crash={r.get('crashed')} :: {json.dumps(r['params'])}")
    json.dump(res[:15], open("sweep_top.json","w"), indent=1)
