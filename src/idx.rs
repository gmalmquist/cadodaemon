use std::borrow::Borrow;

pub trait Index: Clone + Copy + From<usize> + Into<usize> {
}

#[derive(Clone, Debug)]
pub struct IndexedStore<I: Index, T> {
    array: Vec<Option<T>>,
    size: usize,
    zero: I,
}

impl<I: Index, T> IndexedStore<I, T> {
    pub fn new() -> Self {
        Self {
            array: vec![],
            size: 0,
            zero: 0.into(),
        }
    }

    pub fn indices(&self) -> IndexedStoreKeyIter<'_, I, T> {
        IndexedStoreKeyIter {
            store: self,
            index: 0,
        }
    }

    pub fn contains<K: Borrow<I>>(&self, index: K) -> bool {
        self.get(index).is_some()
    }

    pub fn values(&self) -> impl Iterator<Item=&T> {
        self.indices().map(|key| &self[key])
    }

    pub fn push(&mut self, item: T) -> I {
        let idx = I::from(self.array.len());
        self.array.push(Some(item));
        self.size += 1;
        idx
    }

    pub fn get<K: Borrow<I>>(&self, index: K) -> Option<&T> {
        match self.array.get((*index.borrow()).into()) {
            Some(t) => t.as_ref(),
            None => None,
        }
    }

    pub fn set<K: Borrow<I>>(&mut self, index: K, value: T) {
        let i: usize = (*index.borrow()).into();
        while self.array.len() <= i {
            self.array.push(None);
        }
        self.array[i] = Some(value);
    }

    pub fn remove<K: Borrow<I>>(&mut self, index: K) -> Option<T> {
        let item = self.array[(*index.borrow()).into()].take();
        if item.is_some() {
            self.size -= 1;
        }
        item
    }

    pub fn count(&self) -> usize {
        self.size
    }
}

impl<'a, I: Index, T> IntoIterator for &'a IndexedStore<I, T> {
    type Item = I;
    type IntoIter = IndexedStoreKeyIter<'a, I, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.indices()
    }
}

pub struct IndexedStoreKeyIter<'a, I: Index, T> {
    store: &'a IndexedStore<I, T>,
    index: usize,
}

impl<'a, I: Index, T> Iterator for IndexedStoreKeyIter<'a, I, T> {
    type Item = I;

    fn next(&mut self) -> Option<I> {
        while self.index < self.store.array.len() {
            let index = self.index;
            self.index += 1;
            if self.store.array[index].is_some() {
                return Some(I::from(index));
            }
        }
        None
    }
}

impl<K: Borrow<I>, I: Index, T> std::ops::Index<K> for IndexedStore<I, T> {
    type Output = T;

    fn index(&self, index: K) -> &Self::Output {
        self.array[(*index.borrow()).into()].as_ref().unwrap()
    }
}

impl<K: Borrow<I>, I: Index, T> std::ops::IndexMut<K> for IndexedStore<I, T> {
    fn index_mut(&mut self, index: K) -> &mut Self::Output {
        self.array[(*index.borrow()).into()].as_mut().unwrap()
    }
}