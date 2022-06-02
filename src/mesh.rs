use crate::ops::Pivot;
use nalgebra::{Point3, Vector3};
use std::collections::{HashMap, HashSet};
use crate::idx::{Index, IndexedStore};

extern crate derive_more;
use derive_more::{From, Into};

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug, Into, From)]
pub struct Face(usize);
impl Index for Face {}

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug, Into, From)]
pub struct Vertex(usize);
impl Index for Vertex {}

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Corner {
    pub face: Face,
    pub index: usize,
}

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug, Into, From)]
pub struct GroupId(usize);
impl Index for GroupId {}

#[derive(Clone, Debug)]
pub struct Group {
    pub name: String,
    pub faces: Vec<Face>,
}

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug, Into, From)]
pub struct EdgeId(usize);
impl Index for EdgeId {}

#[derive(Clone, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Edge {
    pub src: Vertex,
    pub dst: Vertex,
}

#[derive(Clone, Debug)]
pub struct Mesh {
    pub name: String,
    pub vertices: IndexedStore<Vertex, Point3<f64>>,
    pub edges: IndexedStore<EdgeId, Edge>,
    pub faces: IndexedStore<Face, Vec<Vertex>>,
    pub normals: IndexedStore<Face, Option<Vector3<f64>>>,
    pub groups: IndexedStore<GroupId, Group>,
    faces_to_group: IndexedStore<Face, Option<GroupId>>,
    vertices_to_faces: IndexedStore<Vertex, Vec<Face>>,
    vertices_to_edges: IndexedStore<Vertex, HashMap<Vertex, EdgeId>>,
    edges_to_faces: IndexedStore<EdgeId, Vec<Face>>,
}

pub struct TopologyIter<T> {
    index: usize,
    count: usize,
    func: Box<dyn Fn(usize) -> T>,
}

impl<T> TopologyIter<T> {
    pub fn new(count: usize, func: Box<dyn Fn(usize) -> T>) -> Self {
        Self {
            index: 0,
            count,
            func,
        }
    }
}

impl<T> Iterator for TopologyIter<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.count {
            return None;
        }
        let item = (self.func)(self.index);
        self.index += 1;
        Some(item)
    }
}

impl Mesh {
    pub fn new() -> Self {
        Self {
            name: "".to_string(),
            vertices: IndexedStore::new(),
            edges: IndexedStore::new(),
            faces: IndexedStore::new(),
            normals: IndexedStore::new(),
            groups: IndexedStore::new(),
            faces_to_group: IndexedStore::new(),
            vertices_to_faces: IndexedStore::new(),
            vertices_to_edges: IndexedStore::new(),
            edges_to_faces: IndexedStore::new(),
        }
    }

    pub fn compact(self) -> Self {
        let mut compacted = Mesh::new();
        compacted.name = self.name.clone();
        let mut vertex_remap: HashMap<Vertex, Vertex> = HashMap::new();
        let mut face_remap: HashMap<Face, Face> = HashMap::new();

        for vertex in self.vertices.indices() {
            let nv = compacted.add_vertex(*vertex.to_point(&self), None);
            vertex_remap.insert(vertex, nv);
        }

        for face in self.faces.indices() {
            let nf = compacted.add_face(face.vertices(&self).iter()
                .map(|v| *vertex_remap.get(v).unwrap())
                .collect());
            face_remap.insert(face, nf);
        }

        for edge in self.edges.values() {
            compacted.add_edge(Edge::new(
                *vertex_remap.get(&edge.src).unwrap(),
                *vertex_remap.get(&edge.dst).unwrap(),
            ));
        }

        compacted
    }

    pub fn remove_face(&mut self, face: Face) {
        let vertices = self.faces.remove(face);
        if vertices.is_none() {
            return;
        }
        let vertices = vertices.unwrap();
        self.normals.remove(face);
        let group = self.faces_to_group.remove(face).unwrap();
        if let Some(group) = group {
            self.groups[group].faces.retain(|f| f != &face);
        }
        for v in vertices {
            self.vertices_to_faces[v].retain(|f| f != &face);
            for e in self.vertices_to_edges[v].values() {
                self.edges_to_faces[e].retain(|f| f != &face);
            }
        }
    }

    pub fn remove_edge(&mut self, e: EdgeId) {
        if !self.edges.contains(e) {
            return;
        }
        let edge = e.edge(self).clone();

        self.vertices_to_edges[edge.src].remove(&edge.dst);
        self.vertices_to_edges[edge.dst].remove(&edge.src);

        // Removing an edge destroys faces it is attached to.
        let faces_to_remove = self.edges_to_faces.remove(e).unwrap();
        for f in faces_to_remove {
            self.remove_face(f);
        }

        self.edges.remove(e);
    }

    pub fn remove_vertex(&mut self, v: Vertex) {
        if !self.vertices.contains(v) {
            return;
        }

        // Remove all adjacent edges
        let edges_to_remove = self.vertices_to_edges.remove(v).unwrap();
        for e in edges_to_remove.values() {
            self.remove_edge(*e);
        }

        // Remove all adjacent faces
        self.vertices_to_faces.remove(v);
        self.vertices.remove(v);
    }

    pub fn ungroup(&mut self, gid: GroupId) {
        let group = self.groups.remove(gid);
        if group.is_none() {
            return;
        }
        let group = group.unwrap();

        for face in group.faces {
            self.faces_to_group[face] = None;
        }
    }

    pub fn add_vertex(&mut self, point: Point3<f64>, dedup_eps: Option<f64>) -> Vertex {
        if let Some(eps) = dedup_eps {
            for vi in self.vertices() {
                let v = &self.vertices[vi];
                if nalgebra::distance(v, &point) < eps {
                    return vi;
                }
            }
        }
        let vertex = self.vertices.push(point);
        self.vertices_to_edges.push(HashMap::new());
        self.vertices_to_faces.push(vec![]);
        vertex
    }

    pub fn add_face(&mut self, vertices: Vec<Vertex>) -> Face {
        let face = self.faces.push(vertices);
        self.faces_to_group.push(None);
        face.assert_valid(self);

        for c in face.corners(&self) {
            let a = c.to_vertex(&self);
            self.vertices_to_faces[a].push(face.clone());

            let b = c.next(&self).to_vertex(&self);
            let edge = self.add_edge(Edge::new(a, b));
            self.edges_to_faces[edge].push(face);
        }

        face
    }

    pub fn add_edge(&mut self, edge: Edge) -> EdgeId {
        edge.assert_valid(&self);
        if let Some(eid) = self.vertices_to_edges[edge.src].get(&edge.dst) {
            return eid.clone();
        }
        let eid = self.edges.push(edge);
        let edge = &self.edges[eid];
        self.vertices_to_edges[edge.src].insert(edge.dst, eid);
        self.vertices_to_edges[edge.dst].insert(edge.src, eid);
        self.edges_to_faces.push(vec![]);
        eid
    }

    pub fn add_group(&mut self, group: Group) -> GroupId {
        let gid = self.groups.push(group);
        for face in &self.groups[gid].faces {
            self.faces_to_group[face] = Some(gid);
        }
        gid
    }

    pub fn scale_mesh(&mut self, amount: Vector3<f64>, pivot: Pivot) {
        let pivot: Point3<f64> = match pivot {
            Pivot::Origin => Point3::origin(),
            Pivot::GlobalCentroid => self.centroid(),
            Pivot::Point(p) => p,
            Pivot::IndividualCentroids => self.centroid(),
        };
        let vertices: Vec<Vertex> = self.vertices().collect();
        for v in vertices {
            let pt = v.to_point(&self);
            let delta = *pt - pivot;
            self.vertices[v] = pivot + delta.component_mul(&amount);
        }
    }

    pub fn faces(&self) -> impl Iterator<Item=Face> + '_ {
        self.faces.indices()
    }

    pub fn vertices(&self) -> impl Iterator<Item=Vertex> + '_  {
        self.vertices.indices()
    }

    pub fn edges(&self) -> impl Iterator<Item=EdgeId> + '_  {
        self.edges.indices()
    }

    pub fn centroid(&self) -> Point3<f64> {
        let mut centroid = Point3::origin();
        if self.vertices.count() == 0 {
            return centroid;
        }
        for pt in self.vertices.values() {
            centroid.x += pt.x;
            centroid.y += pt.y;
            centroid.z += pt.z;
        }
        centroid / self.vertices.count() as f64
    }

    pub fn compute_normals(&mut self) {
        self.assert_valid();
        for f in self.faces.indices() {
            self.normals.set(f, Some(f.compute_normal(self)));
        }
    }

    pub fn validate(&self) -> bool {
        for f in self.faces.indices() {
            if !f.is_valid(self) {
                return false;
            }
        }
        true
    }

    pub fn assert_valid(&self) {
        assert!(self.validate());
    }
}

impl Group {
    pub fn new(name: String) -> Self {
        Self {
            name,
            faces: vec![],
        }
    }
}

impl Face {
    pub fn vertices(self, mesh: &Mesh) -> &Vec<Vertex> {
        &mesh.faces[self]
    }

    pub fn corner_count(self, mesh: &Mesh) -> usize {
        mesh.faces[self].len()
    }

    pub fn corners(self, mesh: &Mesh) -> TopologyIter<Corner> {
        TopologyIter::new(
            mesh.faces[self].len(),
            Box::new(move |i| Corner { face: self.clone(), index: i }),
        )
    }

    pub fn first_corner(&self) -> Corner {
        Corner {
            face: self.clone(),
            index: 0,
        }
    }

    pub fn get_normal(self, mesh: &Mesh) -> Option<&Vector3<f64>> {
        match mesh.normals.get(self) {
            Some(n) => n.as_ref(),
            None => None,
        }
    }

    pub fn compute_normal(&self, mesh: &Mesh) -> Vector3<f64> {
        let mut corners = self.corners(mesh);
        let a = corners.next().unwrap().to_point(mesh);
        let b = corners.next().unwrap().to_point(mesh);
        let c = corners.next().unwrap().to_point(mesh);
        let u = b - a;
        let v = c - a;
        let mut n = u.cross(&v);
        if n.norm_squared() == 0. {
            return n;
        }
        n.normalize_mut();
        n
    }

    pub fn assert_valid(&self, mesh: &Mesh) {
        assert!(self.is_valid(mesh));
    }

    pub fn is_valid(&self, mesh: &Mesh) -> bool {
        if !mesh.faces.contains(*self) {
            return false;
        }
        if self.is_degenerate(mesh) {
            return false;
        }
        for v in &mesh.faces[*self] {
            if !v.is_valid(mesh) {
                return false;
            }
        }
        true
    }

    pub fn is_degenerate(&self, mesh: &Mesh) -> bool {
        let corners = &mesh.faces[*self];
        if corners.len() < 3 {
            return true;
        }
        for a in 0..(corners.len() - 1) {
            for b in (a + 1)..corners.len() {
                if corners[a] == corners[b] {
                    return true;
                }
            }
        }
        false
    }
}

impl Vertex {
    fn to_point(self, mesh: &Mesh) -> &Point3<f64> {
        &mesh.vertices[self]
    }

    pub fn is_valid(&self, mesh: &Mesh) -> bool {
        mesh.vertices.contains(*self)
    }

    pub fn assert_valid(&self, mesh: &Mesh) {
        assert!(self.is_valid(mesh));
    }
}

impl Corner {
    fn to_vertex(self, mesh: &Mesh) -> Vertex {
        mesh.faces[self.face][self.index]
    }

    fn next(&self, mesh: &Mesh) -> Corner {
        let index = (self.index + 1) % self.face.corner_count(mesh);
        Self {
            face: self.face,
            index,
        }
    }

    fn prev(&self, mesh: &Mesh) -> Corner {
        let index = if self.index == 0 {
            self.face.corner_count(mesh) - 1
        } else {
            self.index - 1
        };
        Self {
            face: self.face,
            index,
        }
    }

    fn to_point(self, mesh: &Mesh) -> &Point3<f64> {
        self.to_vertex(mesh).to_point(mesh)
    }
}

impl EdgeId {
    pub fn tangent(self, mesh: &Mesh) -> Vector3<f64> {
        let (a, b) = self.points(mesh);
        let mut v: Vector3<f64> = b - a;
        v.normalize_mut();
        v
    }

    pub fn vertices(self, mesh: &Mesh) -> (Vertex, Vertex) {
        let e = self.edge(mesh);
        (e.src, e.dst)
    }

    pub fn points(self, mesh: &Mesh) -> (&Point3<f64>, &Point3<f64>) {
        let (a, b) = self.vertices(mesh);
        (a.to_point(&mesh), b.to_point(mesh))
    }

    pub fn edge(self, mesh: &Mesh) -> &Edge {
        &mesh.edges[self]
    }

    pub fn opposite_face(&self, face: &Face, mesh: &Mesh) -> Option<Face> {
        for f in &mesh.edges_to_faces[*self] {
            if f != face {
                return Some(f.clone());
            }
        }
        None
    }

    pub fn opposite_vertex(self, v: Vertex, mesh: &Mesh) -> Vertex {
        let (src, dst) = self.vertices(mesh);
        if src == v {
            return dst
        }
        src
    }

    pub fn is_valid(&self, mesh: &Mesh) -> bool {
        if !mesh.edges.contains(*self) {
            return false;
        }
        self.edge(mesh).is_valid(mesh)
    }

    pub fn assert_valid(&self, mesh: &Mesh) {
        assert!(self.is_valid(mesh));
    }
}

impl Edge {
    pub fn new(a: Vertex, b: Vertex) -> Self {
        let (a, b) = if a < b {
            (a, b)
        } else {
            (b, a)
        };
        Self {
            src: a,
            dst: b,
        }
    }

    pub fn is_valid(&self, mesh: &Mesh) -> bool {
        self.src != self.dst && self.src.is_valid(&mesh) && self.dst.is_valid(&mesh)
    }

    pub fn assert_valid(&self, mesh: &Mesh) {
        assert!(self.is_valid(mesh));
    }
}

impl From<obj::ObjData> for Mesh {
    fn from(obj: obj::ObjData) -> Self {
        // NB: it might be more correct to load this into multiple meshes?
        // This loses information about sub-objects, though maybe that's metadata that can just
        // be added to the mesh later.
        let mut mesh = Mesh::new();
        for p in obj.position {
            mesh.add_vertex(
                Point3::from_slice(&[p[0] as f64, p[1] as f64, p[2] as f64]),
                None,
            );
        }
        for o in obj.objects {
            for group in o.groups {
                if mesh.name.len() == 0 {
                    mesh.name = group.name.clone();
                }
                let mut group_faces = vec![];
                for poly in group.polys {
                    let mut face = vec![];
                    for obj::IndexTuple(pos, _, _) in poly.0 {
                        face.push(Vertex(pos));
                    }
                    let face = mesh.add_face(face);
                    group_faces.push(face);
                }
                mesh.add_group(Group {
                    name: group.name.clone(),
                    faces: group_faces,
                });
            }
        }
        mesh
    }
}

impl From<obj::Obj> for Mesh {
    fn from(obj: obj::Obj) -> Self {
        obj.data.into()
    }
}

impl From<Mesh> for obj::Obj {
    fn from(mesh: Mesh) -> Self {
        let mesh = mesh.compact();

        let mut used_faces = HashSet::new();
        let mut groups = vec![];
        for group in mesh.groups.values() {
            groups.push(obj::Group {
                name: group.name.clone(),
                index: groups.len(),
                material: None,
                polys: {
                    let mut faces = vec![];
                    for face in mesh.faces() {
                        faces.push(obj::SimplePolygon(
                            face.vertices(&mesh)
                                .iter()
                                .map(|v| obj::IndexTuple(v.0, None, None))
                                .collect(),
                        ));
                        used_faces.insert(face);
                    }
                    faces
                },
            });
        }
        if used_faces.len() < mesh.faces.count() {
            // add default object
            groups.push(obj::Group {
                name: mesh.name.clone(),
                index: groups.len(),
                material: None,
                polys: mesh
                    .faces()
                    .filter(|face| !used_faces.contains(face))
                    .map(|face| {
                        obj::SimplePolygon(
                            face.vertices(&mesh)
                                .iter()
                                .map(|v| obj::IndexTuple(v.0, None, None))
                                .collect(),
                        )
                    })
                    .collect(),
            });
        }

        let data = obj::ObjData {
            position: mesh
                .vertices
                .values()
                .map(|v| [v.x as f32, v.y as f32, v.z as f32])
                .collect(),
            objects: vec![obj::Object {
                name: mesh.name.clone(),
                groups,
            }],
            material_libs: vec![],
            normal: vec![],
            texture: vec![],
        };
        obj::Obj {
            data,
            path: std::path::PathBuf::new(),
        }
    }
}
