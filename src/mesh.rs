use nalgebra::{Point, Point3, Vector3};
use obj::Group;

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Face(usize);

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Vertex(usize);

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Corner {
    pub face: Face,
    pub index: usize,
}

#[derive(Clone, Debug)]
pub struct Mesh {
    pub vertices: Vec<Point3<f64>>,
    pub faces: Vec<Vec<Vertex>>,
    pub normals: Vec<Option<Vector3<f64>>>,
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
            vertices: vec![],
            faces: vec![],
            normals: vec![],
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
        Vertex(self.vertices.len() - 1)
    }

    pub fn add_face(&mut self, vertices: Vec<Vertex>) -> Face {
        self.faces.push(vertices);
        let face = Face(self.faces.len() - 1);
        face
    }

    pub fn faces(&self) -> TopologyIter<Face> {
        TopologyIter::new(self.faces.len(), Box::new(|i| Face(i)))
    }

    pub fn vertices(&self) -> TopologyIter<Vertex> {
        TopologyIter::new(self.vertices.len(), Box::new(|i| Vertex(i)))
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

impl Face {
    pub fn vertices(self, mesh: &Mesh) -> &Vec<Vertex> {
        &mesh.faces[self.0]
    }

    pub fn corner_count(&self, mesh: &Mesh) -> usize {
        mesh.faces[self.0].len()
    }

    pub fn corners(&self, mesh: &Mesh) -> TopologyIter<Corner> {
        let face = self.clone();
        TopologyIter::new(mesh.faces[self.0].len(), Box::new(move |i| Corner {
            face,
            index: i,
        }))
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

impl From<obj::ObjData> for Mesh {
    fn from(obj: obj::ObjData) -> Self {
        // NB: it might be more correct to load this into multiple meshes?
        // This loses information about sub-objects, though maybe that's metadata that can just
        // be added to the mesh later.
        let mut mesh = Mesh::new();
        for p in obj.position {
            mesh.add_vertex(Point3::from_slice(&[p[0] as f64, p[1] as f64, p[2] as f64]), None);
        }
        for o in obj.objects {
            for group in o.groups {
                for poly in group.polys {
                    let mut face = vec![];
                    for obj::IndexTuple(pos, _, _) in poly.0 {
                        face.push(Vertex(pos));
                    }
                    mesh.add_face(face);
                }
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
        let mut data = obj::ObjData {
            position: mesh.vertices.iter()
                .map(|v| [v.x as f32, v.y as f32, v.z as f32])
                .collect(),
            objects: vec![
                obj::Object {
                    name: "mesh".to_string(),
                    groups: vec![obj::Group {
                        name: "mesh".to_string(),
                        index: 0,
                        material: None,
                        polys: mesh.faces()
                            .map(|face| obj::SimplePolygon(
                                face.vertices(&mesh).iter()
                                    .map(|v| obj::IndexTuple(v.0, None, None))
                                    .collect()
                            ))
                            .collect()
                    }]
                }
            ],
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