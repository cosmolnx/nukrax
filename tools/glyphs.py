# Extracts flattened outer contours of the NUKRAX display font glyphs for physics shapes.
import json
from fontTools.ttLib import TTFont
from fontTools.pens.recordingPen import DecomposingRecordingPen
f=TTFont('assets/fonts/nukrax-display.otf'); gs=f.getGlyphSet(); cm=f.getBestCmap(); upm=f['head'].unitsPerEm
def flat(ops,n=6):
    cs=[];c=[];p=None
    for op,a in ops:
        if op=='moveTo': c=[a[0]];p=a[0]
        elif op=='lineTo': c.append(a[0]);p=a[0]
        elif op=='curveTo':
            p0=p;p1,p2,p3=a
            for i in range(1,n+1):
                t=i/n;u=1-t
                c.append((u**3*p0[0]+3*u*u*t*p1[0]+3*u*t*t*p2[0]+t**3*p3[0],u**3*p0[1]+3*u*u*t*p1[1]+3*u*t*t*p2[1]+t**3*p3[1]))
            p=p3
        elif op=='qCurveTo':
            pts=list(a);p0=p
            for k in range(len(pts)-1):
                c1=pts[k];e=pts[k+1] if k==len(pts)-2 else ((pts[k][0]+pts[k+1][0])/2,(pts[k][1]+pts[k+1][1])/2)
                for i in range(1,n+1):
                    t=i/n;u=1-t
                    c.append((u*u*p0[0]+2*u*t*c1[0]+t*t*e[0],u*u*p0[1]+2*u*t*c1[1]+t*t*e[1]))
                p0=e
            p=p0
        elif op in('closePath','endPath'):
            if c: cs.append(c)
            c=[]
    return cs
def area(c): return sum(c[i][0]*c[(i+1)%len(c)][1]-c[(i+1)%len(c)][0]*c[i][1] for i in range(len(c)))/2
out={'upm':upm,'g':{}}
for ch in 'NUKRAX':
    pen=DecomposingRecordingPen(gs); gs[cm[ord(ch)]].draw(pen)
    cs=flat(pen.value); cs.sort(key=lambda c:-abs(area(c)))
    o=cs[0]
    xs=[p[0] for p in o];ys=[p[1] for p in o]
    sg=1 if area(o)>0 else -1
    rest=[[ [round(x),round(y)] for x,y in c] for c in cs[1:] if area(c)*sg<0]
    out['g'][ch]={'adv':gs[cm[ord(ch)]].width,'bb':[min(xs),min(ys),max(xs),max(ys)],'c':[[round(x),round(y)] for x,y in o],'holes':rest}
    print(ch,out['g'][ch]['adv'],out['g'][ch]['bb'],len(o))
open('assets/glyphs.js','w').write('window.NKX_GLYPHS='+json.dumps(out,separators=(',',':'))+';')
