Development Docker container setup
==================================

The goal is to create a docker container setup that is capable of running
entirely offline, including the initial stage of loading the container.

This means we need to have all development libraries and utilities included in
the container. We also needed to export the docker image so it can be easily
imported by the attendees by simply downloading a file from the Raspberry Pis
available on site.

To build the image:

```console
$ docker/build
```

This will build the docker image with a name of `roscon-2023-realtime-workshop`
and save it into a compressed tarball at `docker/image.tar.gz`.

This image can be imported by attendees via `docker/fetch`. See the
[README.md](../README.md) for more details on how to use this image.

TODOS:

- [ ] Networking with the Raspberry Pi without using host networking.
