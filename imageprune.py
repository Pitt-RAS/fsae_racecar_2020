#!/usr/bin/env python
import docker


def main():
    client = docker.from_env()

    images = client.images.list()

    versions = []
    extra_images = []
    for image in images:
        for tag in image.tags:
            split = tag.split(':')
            if split[0] == 'pittras/magellan-2018-base' and len(split) > 1:
                number = split[1].split('-')
                if len(number) != 2:
                    break
                if number[0] != 'master':
                    extra_images.append(split[1])
                    break

                number = number[1]
                versions.append(number)

    versions.sort()

    for version in versions[:-1]:
        image_name = 'pittras/magellan-2018-base:master-{}'.format(version)
        print('Removing {}'.format(image_name))
        client.images.remove(image_name)

    for tag in extra_images:
        image_name = 'pittras/magellan-2018-base:{}'.format(tag)
        print('Removing {}'.format(image_name))
        client.images.remove(image_name)


if __name__ == "__main__":
    main()