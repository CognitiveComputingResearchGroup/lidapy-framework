import random
import string


def generate_random_name(prefix, length):
    nonce_len = length - len(prefix)
    if nonce_len <= 0:
        raise Exception("Invalid name length ({})".format(length))

    return prefix + "".join([random.choice(string.hexdigits) for i in xrange(nonce_len)])
