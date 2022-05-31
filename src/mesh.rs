use nalgebra;
use nalgebra::Vector3;

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Face(usize);

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Vertex(usize);

#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Hash, Debug)]
pub struct Corner {
    pub face: Face,
    pub index: usize,
}

pub struct Mesh {
    pub vertices: Vec<Vector3<f64>>,
    pub faces: Vec<Vec<Vertex>>,
    pub normals: Vec<Option<Vector3<f64>>>
}

pub struct FaceCornersIter {
    face: Face,
    index: usize,
    count: usize,
}

impl Iterator for FaceCornersIter {
    type Item = Corner;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.count {
            return None;
        }
        let index = self.index;
        self.index += 1;
        Some(Corner {
            face: self.face,
            index,
        })
    }
}

impl Mesh {
    fn compute_normals(&mut self) {
        self.assert_valid();
        self.normals.clear();
        for i in 0..self.faces.len() {
            self.normals.push(Some(Face(i).compute_normal(self)));
        }
    }

    fn validate(&self) -> bool {
        for f in 0..self.faces.len() {
            if !Face(f).is_valid(self) {
                return false;
            }
        }
        true
    }

    fn assert_valid(&self) {
        assert!(self.validate());
    }
}

impl Face {
    pub fn to_vertices(self, mesh: &Mesh) -> &Vec<Vertex> {
        &mesh.faces[self.0]
    }

    pub fn corner_count(&self, mesh: &Mesh) -> usize {
        mesh.faces[self.0].len()
    }

    pub fn corners(&self, mesh: &Mesh) -> FaceCornersIter {
        FaceCornersIter {
            face: self.clone(),
            index: 0,
            count: self.corner_count(mesh),
        }
    }

    pub fn first_corner(&self) -> Corner {
        Corner {
            face: self.clone(),
            index: 0,
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
        for a in 0..(corners.len()-1) {
            for b in (a+1)..corners.len() {
                if corners[a] == corners[b] {
                    return true;
                }
            }
        }
        false
    }
}

impl Vertex {
    fn to_point(self, mesh: &Mesh) -> &Vector3<f64> {
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

    fn to_point(self, mesh: &Mesh) -> &Vector3<f64> {
        self.to_vertex(mesh).to_point(mesh)
    }
}
