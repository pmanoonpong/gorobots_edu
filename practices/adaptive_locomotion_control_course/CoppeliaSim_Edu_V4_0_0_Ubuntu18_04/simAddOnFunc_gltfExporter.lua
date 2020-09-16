-- constants:
-- accessor.componentType:
local BYTE=5120
local UNSIGNED_BYTE=5121
local SHORT=5122
local UNSIGNED_SHORT=5123
local UNSIGNED_INT=5125
local FLOAT=5126
-- primitive.mode:
local POINTS=0
local LINES=1
local LINE_LOOP=2
local LINE_STRIP=3
local TRIANGLES=4
local TRIANGLE_STRIP=5
local TRIANGLE_FAN=6

function b64enc(data)
    return sim.transformBuffer(data,sim.buffer_uint8,1,0,sim.buffer_base64)
end

function getGLTFMatrix(h,rel)
    local matr=sim.getObjectMatrix(h,rel)
    return {
        -- matrix in glTF is column major
        matr[1], matr[5], matr[9], 0,
        matr[2], matr[6], matr[10], 0,
        matr[3], matr[7], matr[11], 0,
        matr[4], matr[8], matr[12], 1
    }
end

function minmax(v,o,s)
    local m=0
    local M=0
    for i=o,#v,s do
        if v[i]~=nil then
            if i==1 or v[i]<m then m=v[i] end
            if i==1 or v[i]>M then M=v[i] end
        end
    end
    return m,M
end

function embedMesh(doc,h)
    local vertices,indices=sim.getShapeMesh(h)
    local nv=#vertices/3
    local ni=#indices
    local nt=#indices/3
    local ibv=#doc.buffers+0
    local ibi=#doc.buffers+1
    local dv=sim.packFloatTable(vertices)
    local di=sim.packUInt16Table(indices)
    table.insert(doc.buffers,{byteLength=#dv, uri='data:application/octet-stream;base64,'..b64enc(dv)})
    table.insert(doc.buffers,{byteLength=#di, uri='data:application/octet-stream;base64,'..b64enc(di)})
    table.insert(doc.bufferViews,{buffer=ibv, byteLength=#dv, byteOffset=0})
    table.insert(doc.bufferViews,{buffer=ibi, byteLength=#di, byteOffset=0})
    local vxmin,vxmax=minmax(vertices,1,3)
    local vymin,vymax=minmax(vertices,2,3)
    local vzmin,vzmax=minmax(vertices,3,3)
    local imin,imax=minmax(indices,1,1)
    table.insert(doc.accessors,{bufferView=ibv, byteOffset=0, componentType=FLOAT, type='VEC3', count=nv, min={vxmin,vymin,vzmin}, max={vxmax,vymax,vzmax}})
    table.insert(doc.accessors,{bufferView=ibi, byteOffset=0, componentType=UNSIGNED_SHORT, type='SCALAR', count=ni, min={imin}, max={imax}})
    local i=#doc.meshes
    table.insert(doc.meshes,{primitives={{attributes={POSITION=ibv},indices=ibi,mode=TRIANGLES}}})
    return i
end

function embedBasicMaterial(doc,col1,col2)
    local i=#doc.materials
    table.insert(doc.materials,{
        name="Material",
        pbrMetallicRoughness={
            baseColorFactor={col1[1], col1[2], col1[3], 1.0},
            metallicFactor=0.1,
            roughnessFactor=0.5
        }
    })
    return i
end

function embedTexture(doc,imgdata)
    local i=#doc.images
    table.insert(doc.images,{uri='data:application/octet-stream;base64,'..b64enc(imgdata)})
    return i
end

function export_glTF(fileName)
	local file=io.open(fileName,"w")
	local extension
    local doc={}
    doc.asset={
        version='2.0',
        generator='CoppeliaSim glTF Exporter'
    }
    doc.buffers={}
    doc.bufferViews={}
    doc.accessors={}
    doc.nodes={}
    doc.meshes={}
    doc.samplers={{magFilter=9729, minFilter=9987, wrapS=33648, wrapT=33648}}
    --doc.images={}
    --doc.textures={}
    doc.materials={}
	local selectedObjects=sim.getObjectSelection()
	local allObjects=sim.getObjectsInTree(sim.handle_scene)
	if selectedObjects and #selectedObjects==1 then
		allObjects=sim.getObjectsInTree(selectedObjects[1])
	end
	local allIndividualShapesToRemove={}
	local visibleLayers=sim.getInt32Parameter(sim.intparam_visible_layers)
    local rootNode={children={}}
    table.insert(doc.nodes,rootNode)
	for i,obj in pairs(allObjects) do
		local objType=sim.getObjectType(obj)
		local objName=sim.getObjectName(obj)
		local parentName="*"
		local parentHandle=sim.getObjectParent(obj)
		if parentHandle~=-1 then
			parentName=sim.getObjectName(parentHandle)
		end
		local res,layers=sim.getObjectInt32Parameter(obj,10)
        local visible=sim.boolAnd32(visibleLayers,layers)~=0
        if visible then
            if objType==sim.object_shape_type then
                local node={}
                table.insert(doc.nodes,node)
                local nodeIndex=#doc.nodes-1
                table.insert(rootNode.children,nodeIndex)
                node.name=objName
                node.matrix=getGLTFMatrix(obj,-1)
                local res,param=sim.getObjectInt32Parameter(obj,sim.shapeintparam_compound)
                if param~=0 then
                    node.children={}
                    local tobj=sim.copyPasteObjects({obj},0)
                    local individualShapes=sim.ungroupShape(tobj[1])
                    for j,subObj in pairs(individualShapes) do
                        table.insert(allIndividualShapesToRemove,subObj)
                        local subNode={}
                        table.insert(doc.nodes, subNode)
                        local subNodeIndex=#doc.nodes-1
                        table.insert(node.children,subNodeIndex)
                        subNode.name=sim.getObjectName(subObj)
                        subNode.matrix=getGLTFMatrix(subObj,obj)
                        local result,col1=sim.getShapeColor(subObj,nil,sim.colorcomponent_ambient_diffuse)
                        local result,col2=sim.getShapeColor(subObj,nil,sim.colorcomponent_specular)
                        subNode.mesh=embedMesh(doc,subObj)
                        doc.meshes[subNode.mesh+1].primitives[1].material=embedBasicMaterial(doc,col1,col2)
                    end
                else
                    local result,col1=sim.getShapeColor(obj,nil,sim.colorcomponent_ambient_diffuse)
                    local result,col2=sim.getShapeColor(obj,nil,sim.colorcomponent_specular)
                    node.mesh=embedMesh(doc,obj)
                    doc.meshes[node.mesh+1].primitives[1].material=embedBasicMaterial(doc,col1,col2)
                end
            end
        end
    end
    doc.scenes={
        {
            name='Default Scene',
            nodes={0}
        }
    }
    doc.scene=0 -- 0-based index in glTF!
	for i,shape in pairs(allIndividualShapesToRemove) do
		sim.removeObject(shape)
	end
    local json=require("dkjson")
    file:write(json.encode(doc))
	file:close()
end

local appPath=sim.getStringParameter(sim.stringparam_application_path)
local fileName=sim.fileDialog(sim.filedlg_type_save,'Export to glTF...',appPath,'scene.gltf','glTF file','gltf')
if fileName~='' then
    export_glTF(fileName,true)
end
