rule corridorcamera : 
{
        strings:
                $str = "CORRIDOR CAMERA: SEQ 111"

        condition:
                $str
}
