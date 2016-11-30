import random
import string


# TODO: This needs to be enhanced into a name service that guarantees a
# TODO: unique name across multiple processes.
def generate_random_name(prefix, length):
    nonce_len = length - len(prefix)
    if nonce_len <= 0:
        raise Exception("Invalid name length ({})".format(length))

    return prefix + "".join([random.choice(string.hexdigits) for i in xrange(nonce_len)])


def create_class_instance(fully_qualified_class_name, *args, **kwargs):
    """
    Instantiates an object of type given by fully_qualified_class_name, and
    passes the supplied args and kwargs to the class's init method.

    :param fully_qualified_class_name: the fully qualified name of the class to instantiate
    :param args: positional arguments to pass to the class's init method
    :param kwargs: keyword arguments to pass to the class's init method
    :return:
    """
    try:
        tokens = fully_qualified_class_name.split('.')
        module_name = '.'.join(tokens[:-1])
        class_name = tokens[-1]
        module = __import__(module_name)
        for token in module_name.split('.')[1:]:
            module = getattr(module, token)

        callable = getattr(module, class_name)
        instance = callable(*args, **kwargs)
    except Exception as e:
        raise Exception("Failed to construct instance of class {}".format(fully_qualified_class_name))

    return instance
