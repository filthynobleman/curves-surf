import bpy


OutputFile = "//samples.txt"


Obj = bpy.context.active_object
bpy.ops.object.mode_set(mode='OBJECT')

Verts = []
for i in range(len(Obj.data.vertices)):
    v = Obj.data.vertices[i]
    if v.select:
        Verts.append(i)
        
        
with open(bpy.path.abspath(OutputFile), 'w') as fp:
    for v in Verts:
        fp.write("{0}\n".format(v))