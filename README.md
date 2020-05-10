This repository contains a proof of concept of building a simple kernel
module for [illumos](https://illumos.org). Effectively, this shows how
one could implement the simple
[pchtemp(7D)](https://illumos.org/man/7d/pchtemp) driver in
[rust](https://www.rust-lang.org/). Note, this has been done the max
power way -- the wrong way, but faster. Along the way, you'll also find
that this actually manages to create a DTrace SDT probe point, because
after all, what's the symbol table between friends?
