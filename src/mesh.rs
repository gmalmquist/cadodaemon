use crate::ops::Pivot;
use nalgebra::{Point3, Vector3};
use std::collections::{HashMap, HashSet};

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Face(usize);

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Vertex(usize);

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Corner {
    pub face: Face,
    pub index: usize,
}

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct GroupId(usize);

#[derive(Clone, Debug)]
pub struct Group {
    pub name: String,
    pub faces: Vec<Face>,
}

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct EdgeId(usize);

#[derive(Clone, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Edge {
    pub src: Vertex,
    pub dst: Vertex,
}

#[derive(Clone, Debug)]
pub struct Mesh {
    pub name: String,
    pub vertices: Vec<Point3<f64>>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Vec<Vertex>>,
    pub normals: Vec<Option<Vector3<f64>>>,
    pub groups: Vec<Group>,
    faces_to_group: Vec<Option<GroupId>>,
    vertices_to_faces: Vec<Vec<Face>>,
    vertices_to_edges: Vec<HashMap<Vertex, EdgeId>>,
    edges_to_faces: Vec<Vec<Face>>,
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
            vertices: vec![],
            edges: vec![],
            faces: vec![],
            normals: vec![],
            groups: vec![],
            faces_to_group: vec![],
            vertices_to_faces: vec![],
            vertices_to_edges: vec![],
            edges_to_faces: vec![],
        }
    }

    pub fn add_vertex(&mut self, point: Point3<f64>, dedup_eps: Option<f64>) -> Vertex {
        if let Some(eps) = dedup_eps {
            for vi in 0..self.vertices.len() {
                let v = &self.vertices[vi];
                if nalgebra::distance(v, &point) < eps {
                    return Vertex(vi);
                }
            }
        }
        self.vertices.push(point);
        self.vertices_to_edges.push(HashMap::new());
        self.vertices_to_faces.push(vec![]);
        Vertex(self.vertices.len() - 1)
    }

    pub fn add_face(&mut self, vertices: Vec<Vertex>) -> Face {
        self.faces.push(vertices);
        self.faces_to_group.push(None);
        let face = Face(self.faces.len() - 1);

        for c in face.corners(&self) {
            let a = c.to_vertex(&self);
            self.vertices_to_faces[a.0].push(face.clone());

            let b = c.next(&self).to_vertex(&self);
            let edge = self.add_edge(Edge::new(a, b));
            self.edges_to_faces[edge.0].push(face);
        }

        face
    }

    pub fn add_edge(&mut self, edge: Edge) -> EdgeId {
        edge.assert_valid(&self);
        if let Some(eid) = self.vertices_to_edges[edge.src.0].get(&edge.dst) {
            return eid.clone();
        }
        let eid = EdgeId(self.edges.len());
        self.vertices_to_edges[edge.src.0].insert(edge.dst, eid);
        self.vertices_to_edges[edge.dst.0].insert(edge.src, eid);
        self.edges.push(edge);
        self.edges_to_faces.push(vec![]);
        eid
    }

    pub fn add_group(&mut self, group: Group) -> GroupId {
        self.groups.push(group);
        let gid = GroupId(self.groups.len() - 1);
        for face in &self.groups[gid.0].faces {
            self.faces_to_group[face.0] = Some(gid);
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
        for pt in &mut self.vertices {
            let delta = *pt - pivot;
            *pt = pivot + (delta.component_mul(&amount));
        }
    }

    pub fn faces(&self) -> TopologyIter<Face> {
        TopologyIter::new(self.faces.len(), Box::new(|i| Face(i)))
    }

    pub fn vertices(&self) -> TopologyIter<Vertex> {
        TopologyIter::new(self.vertices.len(), Box::new(|i| Vertex(i)))
    }

    pub fn edges(&self) -> TopologyIter<EdgeId> {
        TopologyIter::new(self.edges.len(), Box::new(|i| EdgeId(i)))
    }

    pub fn centroid(&self) -> Point3<f64> {
        let mut centroid = Point3::origin();
        if self.vertices.len() == 0 {
            return centroid;
        }
        for pt in &self.vertices {
            centroid.x += pt.x;
            centroid.y += pt.y;
            centroid.z += pt.z;
        }
        centroid / self.vertices.len() as f64
    }

    pub fn compute_normals(&mut self) {
        self.assert_valid();
        self.normals.clear();
        for i in 0..self.faces.len() {
            self.normals.push(Some(Face(i).compute_normal(self)));
        }
    }

    pub fn validate(&self) -> bool {
        for f in 0..self.faces.len() {
            if !Face(f).is_valid(self) {
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
        &mesh.faces[self.0]
    }

    pub fn corner_count(&self, mesh: &Mesh) -> usize {
        mesh.faces[self.0].len()
    }

    pub fn corners(&self, mesh: &Mesh) -> TopologyIter<Corner> {
        let face = self.clone();
        TopologyIter::new(
            mesh.faces[self.0].len(),
            Box::new(move |i| Corner { face, index: i }),
        )
    }

    pub fn first_corner(&self) -> Corner {
        Corner {
            face: self.clone(),
            index: 0,
        }
    }

    pub fn get_normal<'m>(&self, mesh: &'m Mesh) -> Option<&'m Vector3<f64>> {
        match mesh.normals.get(self.0) {
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
        if self.is_degenerate(mesh) {
            return false;
        }
        for v in &mesh.faces[self.0] {
            if !v.is_valid(mesh) {
                return false;
            }
        }
        true
    }

    pub fn is_degenerate(&self, mesh: &Mesh) -> bool {
        let corners = &mesh.faces[self.0];
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
        &mesh.vertices[self.0]
    }

    pub fn is_valid(&self, mesh: &Mesh) -> bool {
        self.0 < mesh.vertices.len()
    }

    pub fn assert_valid(&self, mesh: &Mesh) {
        assert!(self.is_valid(mesh));
    }
}

impl Corner {
    fn to_vertex(self, mesh: &Mesh) -> Vertex {
        mesh.faces[self.face.0][self.index]
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
        &mesh.edges[self.0]
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

    pub fn other(&self, v: Vertex) -> Vertex {
        if self.src == v {
            self.dst
        } else if self.dst == v {
            self.src
        } else {
            panic!("input vertex does not belong to this edge!")
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
        let mut used_faces = HashSet::new();
        let mut groups = vec![];
        for group in &mesh.groups {
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
        if used_faces.len() < mesh.faces.len() {
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
                .iter()
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
