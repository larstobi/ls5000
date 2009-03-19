inherit eutils git

DESCRIPTION="SANE backend for Nikon Coolscan 5000ED 35mm film scanner"
HOMEPAGE="http://johannes.sipsolutions.net/Projects/ls5000"

EGIT_REPO_URI="http://git.sipsolutions.net/ls5000.git"
#git clone http://git.sipsolutions.net/ls5000.git
LICENSE="GPL-2"
SLOT="0"
KEYWORDS="~amd64 ~ppc ~x86"

DEPEND="media-gfx/sane-backends
		dev-libs/libusb"

src_unpack() {
	git_src_unpack
	cd ${S}
	epatch ${FILESDIR}/Makefile.patch
}

src_compile() {
	emake || die "make failed"
}

src_install() {
	make prefix=${D}/usr install || die "Failed to install ls5000"

	dodir /etc/sane.d
	insinto /etc/sane.d
	doins  ${FILESDIR}/dll.conf

	dodoc README COPYING
}
