

//Manchester encoding and decoding

/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
  * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
  * Implementation of the table-driven Manchester encoding and decoding.
 * author:  * Adam Dunkels <adam@sics.se>
 */
//ex) 00 =>0x5555: 01 01 01 01 01 01 01 01
//ex) ff =>aaaa : 10 10 10 10 10 10 10 10
const unsigned short me_encode_tab[256] = {
0x5555, 0x5556, 0x5559, 0x555a, 0x5565, 0x5566, 0x5569, 0x556a, 0x5595,
0x5596, 0x5599, 0x559a, 0x55a5, 0x55a6, 0x55a9, 0x55aa, 0x5655, 0x5656,
0x5659, 0x565a, 0x5665, 0x5666, 0x5669, 0x566a, 0x5695, 0x5696, 0x5699,
0x569a, 0x56a5, 0x56a6, 0x56a9, 0x56aa, 0x5955, 0x5956, 0x5959, 0x595a,
0x5965, 0x5966, 0x5969, 0x596a, 0x5995, 0x5996, 0x5999, 0x599a, 0x59a5,
0x59a6, 0x59a9, 0x59aa, 0x5a55, 0x5a56, 0x5a59, 0x5a5a, 0x5a65, 0x5a66,
0x5a69, 0x5a6a, 0x5a95, 0x5a96, 0x5a99, 0x5a9a, 0x5aa5, 0x5aa6, 0x5aa9,
0x5aaa, 0x6555, 0x6556, 0x6559, 0x655a, 0x6565, 0x6566, 0x6569, 0x656a,
0x6595, 0x6596, 0x6599, 0x659a, 0x65a5, 0x65a6, 0x65a9, 0x65aa, 0x6655,
0x6656, 0x6659, 0x665a, 0x6665, 0x6666, 0x6669, 0x666a, 0x6695, 0x6696,
0x6699, 0x669a, 0x66a5, 0x66a6, 0x66a9, 0x66aa, 0x6955, 0x6956, 0x6959,
0x695a, 0x6965, 0x6966, 0x6969, 0x696a, 0x6995, 0x6996, 0x6999, 0x699a,
0x69a5, 0x69a6, 0x69a9, 0x69aa, 0x6a55, 0x6a56, 0x6a59, 0x6a5a, 0x6a65,
0x6a66, 0x6a69, 0x6a6a, 0x6a95, 0x6a96, 0x6a99, 0x6a9a, 0x6aa5, 0x6aa6,
0x6aa9, 0x6aaa, 0x9555, 0x9556, 0x9559, 0x955a, 0x9565, 0x9566, 0x9569,
0x956a, 0x9595, 0x9596, 0x9599, 0x959a, 0x95a5, 0x95a6, 0x95a9, 0x95aa,
0x9655, 0x9656, 0x9659, 0x965a, 0x9665, 0x9666, 0x9669, 0x966a, 0x9695,
0x9696, 0x9699, 0x969a, 0x96a5, 0x96a6, 0x96a9, 0x96aa, 0x9955, 0x9956,
0x9959, 0x995a, 0x9965, 0x9966, 0x9969, 0x996a, 0x9995, 0x9996, 0x9999,
0x999a, 0x99a5, 0x99a6, 0x99a9, 0x99aa, 0x9a55, 0x9a56, 0x9a59, 0x9a5a,
0x9a65, 0x9a66, 0x9a69, 0x9a6a, 0x9a95, 0x9a96, 0x9a99, 0x9a9a, 0x9aa5,
0x9aa6, 0x9aa9, 0x9aaa, 0xa555, 0xa556, 0xa559, 0xa55a, 0xa565, 0xa566,
0xa569, 0xa56a, 0xa595, 0xa596, 0xa599, 0xa59a, 0xa5a5, 0xa5a6, 0xa5a9,
0xa5aa, 0xa655, 0xa656, 0xa659, 0xa65a, 0xa665, 0xa666, 0xa669, 0xa66a,
0xa695, 0xa696, 0xa699, 0xa69a, 0xa6a5, 0xa6a6, 0xa6a9, 0xa6aa, 0xa955,
0xa956, 0xa959, 0xa95a, 0xa965, 0xa966, 0xa969, 0xa96a, 0xa995, 0xa996,
0xa999, 0xa99a, 0xa9a5, 0xa9a6, 0xa9a9, 0xa9aa, 0xaa55, 0xaa56, 0xaa59,
0xaa5a, 0xaa65, 0xaa66, 0xaa69, 0xaa6a, 0xaa95, 0xaa96, 0xaa99, 0xaa9a,
0xaaa5, 0xaaa6, 0xaaa9, 0xaaaa, };

//ex) rcvd 01 01 01 01 ==> 0x55 ==> 0x0
//ex) rcvd 10 10 10 10 ==> 0xaa ==> 0xf
const unsigned char me_decode_tab[256] = {
0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1, 0x2, 0x2, 0x3, 0x3, 0x2, 0x2, 0x3, 0x3,
0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1, 0x2, 0x2, 0x3, 0x3, 0x2, 0x2, 0x3, 0x3,
0x4, 0x4, 0x5, 0x5, 0x4, 0x4, 0x5, 0x5, 0x6, 0x6, 0x7, 0x7, 0x6, 0x6, 0x7, 0x7,
0x4, 0x4, 0x5, 0x5, 0x4, 0x4, 0x5, 0x5, 0x6, 0x6, 0x7, 0x7, 0x6, 0x6, 0x7, 0x7,
0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1, 0x2, 0x2, 0x3, 0x3, 0x2, 0x2, 0x3, 0x3,
0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1, 0x2, 0x2,0x3, 0x3, 0x2, 0x2, 0x3, 0x3,
0x4, 0x4, 0x5, 0x5, 0x4, 0x4, 0x5, 0x5, 0x6, 0x6, 0x7, 0x7, 0x6, 0x6, 0x7, 0x7,
0x4, 0x4, 0x5, 0x5, 0x4, 0x4, 0x5, 0x5, 0x6, 0x6, 0x7, 0x7, 0x6, 0x6, 0x7, 0x7,
0x8, 0x8, 0x9, 0x9, 0x8, 0x8, 0x9, 0x9, 0xa, 0xa, 0xb, 0xb, 0xa, 0xa, 0xb, 0xb,
0x8, 0x8, 0x9, 0x9, 0x8, 0x8, 0x9, 0x9, 0xa, 0xa, 0xb, 0xb, 0xa, 0xa, 0xb, 0xb,
0xc, 0xc, 0xd, 0xd, 0xc, 0xc, 0xd, 0xd, 0xe, 0xe, 0xf, 0xf, 0xe, 0xe, 0xf, 0xf,
0xc, 0xc, 0xd, 0xd, 0xc, 0xc, 0xd, 0xd, 0xe, 0xe, 0xf, 0xf, 0xe, 0xe, 0xf, 0xf,
0x8, 0x8, 0x9, 0x9, 0x8, 0x8, 0x9, 0x9, 0xa, 0xa, 0xb, 0xb, 0xa, 0xa, 0xb, 0xb,
0x8, 0x8, 0x9, 0x9, 0x8, 0x8, 0x9, 0x9, 0xa, 0xa, 0xb, 0xb, 0xa, 0xa, 0xb, 0xb,
0xc, 0xc, 0xd, 0xd, 0xc, 0xc, 0xd, 0xd, 0xe, 0xe, 0xf, 0xf, 0xe, 0xe, 0xf, 0xf,
0xc, 0xc, 0xd, 0xd, 0xc, 0xc, 0xd, 0xd, 0xe, 0xe, 0xf, 0xf, 0xe, 0xe, 0xf, 0xf, };

const unsigned char me_valid_tab[256] = {
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1,
0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0,
0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0, 0x1, 0x1,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, };

extern const unsigned short me_encode_tab[256];
extern const unsigned char  me_decode_tab[256];
extern const unsigned char  me_valid_tab[256];

unsigned char  me_valid(unsigned char m);
unsigned short me_encode(unsigned char c);
unsigned char  me_decode16(unsigned short m);
unsigned char  me_decode8(unsigned char m);

/*--------------------------------------------------------------------------
 * Manchester encode an 8-bit byte.
 * This function Manchester encodes an 8-bit byte into a 16-bit word. The function me_decode() does the inverse operation.
 * c : The byte to be encoded
 * retval The encoded word.
--------------------------------------------------------------------------*/
unsigned short me_encode(unsigned char c){
  return me_encode_tab[c];
}
/*---------------------------------------------------------------------------
 * Decode a Manchester encoded 16-bit word.
 * This function decodes a Manchester encoded 16-bit word into a 8-bit byte.
 * The function does not check for parity errors in the encoded byte.
 * \param m The 16-bit Manchester encoded word
 * \return The decoded 8-bit byte
 ---------------------------------------------------------------------------*/
unsigned char me_decode16(unsigned short m)
{
  unsigned char m1, m2, c;

  m1 = m >> 8;
  m2 = m & 0xff;

  c = (me_decode_tab[m1] << 4) |
  me_decode_tab[m2];
  return c;
}
/*---------------------------------------------------------------------------
 * Decode a Manchester encoded 8-bit byte.
 * This function decodes a Manchester encoded 8-bit byte into 4 decoded bits.
 * The function does not check for parity errors in the encoded byte.
 * \param m The 8-bit Manchester encoded byte
 * \return The decoded 4 bits
--------------------------------------------------------------------------*/
unsigned char me_decode8(unsigned char m)
{
  return  me_decode_tab[m];
}
/*---------------------------------------------------------------------------
 * Check if an encoded byte is valid.
 /*---------------------------------------------------------------------------*/
unsigned char me_valid(unsigned char m)
{
  return me_valid_tab[m];
}

void ManchesterLoop(){

	//Your Job.

}
//==================HDB3==================================================================================================
/*  ANSI C Source Code
 *  Code_HDB3.c
 *  Leon 5.11.2008
 */
struct Code_control
{
	int z_num;					//1.젯0돨몸鑒
	int pole;					//2.唐槻섐昑，렷V,B，뎃관윅긴B빈，唐槻섐昑닒V역迦돨헙워
	int V;						//3.V돨섐昑，凜槨V冷狼슥競
	int V_num;					//4.좃몸V裂쇌돨 +-1몸鑒，鹿긱隣펜탉털뙤
	int V_frist_z_define;		//5-8槨긴B륩蛟 角뤠角V鬼뙈櫓돨뒤寧몸，섦V돨뒤寧몸쥐
	int uni_pole;				//6.품충寧몸렷쥐륜뵀，관윅+-1뵨+-V，鱗槨긴B돨弩앴，B宅품寧몸렷쥐禱羹宮럽
	int frist_z_pos;			//7.긴B돨貫零
	int frist_z_pole;			//8.긴B돨令
};

struct Decode_control
{
	int pole;					//1.품寧몸렷쥐섐昑.
};

void Code_HDB(int *,int *,struct Code_control *);
void Decode_HDB(int *,int *,struct Decode_control *);
void PrintHDB3(char s[20], int *);

//     털1뻘角0
//      1 角0
//          1.1 쥐돨낀똑鬼黨3
//              1.1.1  角뒤寧몸0깻할V_frist_z_define=1(鬼뙈쟁충돨뒤寧몸쥐)
//                      frist_z_pos=num;frist_z_pole=-uni_pole;V_frist_z_define=0;
//              code=0;z_num++
//          1.2 쥐돨낀똑角3
//              1.2.1   흔벎V꼇된黨+-1,橙V=-pole;//侶角놓迦
//              code=-V;
//              V=-V;
//              V_frist_z_define=1;
//              z_num=0;
//              1.2.2   흔벎V_num角탉鑒
//                      in[frist_z_pos]=frist_z_pole;pole=V;
//          if(code!=0) uni_pole;
//          in[num]=code;
//      2 角1
//          2.1 code=-pole;
//              pole=code;
//              uni_pole=code;
//          in[num]=code;
void Code_HDB(int *in,int *out,struct Code_control *state)
{
    int num=0;
    int code;
    state->z_num=0;  //역迦珂0돨몸鑒槨0

    while(in[num]!=2)
    {
        if(in[num]==0)
        {
            if(state->z_num<3)
            {
                if((state->z_num==0)&&(state->V_frist_z_define==1))
                {
                    state->frist_z_pos=num;
                    state->frist_z_pole=-state->uni_pole;
                    state->V_frist_z_define=0;
                }
                code=0;
                state->z_num++;
            }
            else
            {
                if((state->V!=1)&&(state->V!=-1)) state->V=-state->pole; //놓迦V
                code=-state->V;
                state->V*=-1;
                state->V_frist_z_define=1;
                state->z_num=0;
                if(((state->V_num)%2==0)&&(state->V_num>=0)) {out[state->frist_z_pos]=state->frist_z_pole;state->pole=state->V;}
                state->V_num=0;
            }
            if(code!=0) state->uni_pole=code;
            out[num]=code;
        }
        else
        {
            code=-state->pole;
            state->pole*=-1;
            state->uni_pole=code;
            state->z_num=0;
            state->V_num++;
            out[num]=code;
        }
        num++;
    }
    out[num]=2;
}
void PrintHDB3(char s[20],int *bits)
{
    int i=0;
    UARTprintf("%s\n",s);
    while(bits[i]!=2)
    {
        if(bits[i]==1)
            UARTprintf(" +%d",bits[i]);
        else
        	UARTprintf("%3d",bits[i]);
        i++;
    }
    UARTprintf("\n");
}

void Hdb3Loop( void )
{
    int num=0;
    struct Code_control c_ctrl,*c_state;
    struct Decode_control d_ctrl,*d_state;

    int bitlen=100;
    int bit,*in,*code,*decode;
    in=(int *)calloc(bitlen,sizeof(int));
    code=(int *)calloc(bitlen,sizeof(int));
    decode=(int *)calloc(bitlen,sizeof(int));

    c_state=&c_ctrl;
    d_state=&d_ctrl;
    c_state->pole=1;         //솝뒤寧몸1 ,놓迦섐昑옵踞긱
    d_state->pole=c_state->pole;

    //뗍俚쌘
    while((bit=getchar())!='\n') //TBD
    {
        bit=bit-48;
        if((bit!=0)&&(bit!=1))
        {
        	UARTprintf("error!");
            exit(1);
        }
        in[num++]=bit;
    }
    in[num++]=2;    //써監륜
    PrintHDB3("the input string",in);

    //HDB3긍쯤
    Code_HDB(in,code,c_state);
    PrintHDB3("coded string",code);

    //HDB3陋쯤
    Decode_HDB(code,decode,d_state);
    PrintHDB3("decoded string",decode);

}



