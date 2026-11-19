
int main(void){
 VSP_Axis *a=&vsp_axis[0];unsigned c;previous_poll_test();
 memset(a,0,sizeof(*a));memset(&vsp_group,0,sizeof(vsp_group));flags=3;
 vsp_group.command.kind=SPF_CMD_PICK;vsp_group.command.goal.extension_mm=65;
 a->before=100;a->goal=a->current=0;queries=0;
 CK(VSP_Poll(0,SPF_EXT)==0 && a->samples==1 && a->phase==0 && queries==3);
 CK(VSP_Poll(0,SPF_EXT)==1 && a->samples==2 && queries==6);
 for(c=0;c<3;c++){
  memset(a,0,sizeof(*a));a->before=100;queries=0;
  miss_position=c==1;miss_command=c==0?0x3a:(c==2?0x35:0);
  CK(VSP_Poll(0,SPF_EXT)==0 && a->samples==0 && queries==c+1);
  miss_command=miss_position=0;
 }
 memset(a,0,sizeof(*a));flags=13;queries=0;
 CK(VSP_Poll(0,SPF_EXT)==2 && queries==1);
 memset(a,0,sizeof(*a));flags=3;active=0;queries=0;
 CK(VSP_Poll(0,SPF_EXT)==0 && a->phase==1 && queries==1);
 printf("PASS actual compact poll: %u checks; actual queries/two confirmations/missing reads/hard fault/legacy caller\n",checks);
 return 0;
}
