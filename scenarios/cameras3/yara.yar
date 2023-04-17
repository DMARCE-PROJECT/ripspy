rule corridorcamera : 
{
        strings:
                $str = "SENDING: 'CORRIDOR CAMERA: SEQ 111'"

        condition:
                $str
}