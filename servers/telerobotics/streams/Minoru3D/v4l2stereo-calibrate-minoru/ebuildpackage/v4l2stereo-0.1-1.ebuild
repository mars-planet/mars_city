# $Header: $

EAPI=5

inherit git-2

DESCRIPTION="Enables the generation of depth maps from stereo vision"
HOMEPAGE="https://github.com/fuzzgun/v4l2stereo"
EGIT_REPO_URI="https://github.com/fuzzgun/v4l2stereo.git"
LICENSE="GPL3"
SLOT="0"
KEYWORDS="x86"
DEPEND="dev-libs/popt"
RDEPEND="${DEPEND}"

src_configure() {
    econf --with-popt
}

src_compile() { :; }

src_install() {
    emake DESTDIR="${D}" PREFIX="/usr" install
    # Install README and (Debian) changelog
    dodoc README.md debian/changelog
}
