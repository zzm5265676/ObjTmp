#include "vertex.hxx"
#include "edge.hxx"

bool Vertex::EdgeEqualByVertexIndex::operator()(
	const Edge* a, const Edge* b) const noexcept {
	int aBeginIndex = a->v0()->index;
	int aEndIndex = a->v1()->index;
	int bBeginIndex = b->v0()->index;
	int bEndIndex = b->v1()->index;
	bool b1 = aBeginIndex == bBeginIndex && aEndIndex == bEndIndex;
	bool b2 = aBeginIndex == bEndIndex && aEndIndex == bBeginIndex;
	return b1 || b2;
}
