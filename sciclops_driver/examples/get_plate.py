



from sciclops_driver import SCICLOPS



command = 'get_plate'
var = {'loc':'tower1', 
    lid=False
    }



test_instrument = SCICLOPS()

test_instrument.get_plate(var['loc'])

