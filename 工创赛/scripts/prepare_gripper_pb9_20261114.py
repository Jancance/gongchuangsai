"""User-requested PB9/TIM4_CH4 mapping; retain radar channels and motion baseline."""
from pathlib import Path
import shutil

R=Path(__file__).resolve().parents[1]
B=R/'tmp/source_top_pick_lead8_20261114/project'
P=R/'tmp/source_top_pick_pb9_20261114/project'

def patch(b):
    newline=b'\r\n' if b'\r\n' in b else b'\n'
    for before,after in [(b'GPIOA',b'GPIOB'),(b'GPIO_Pin_0',b'GPIO_Pin_9'),
                        (b'GPIO_PinSource0',b'GPIO_PinSource9'),(b'TIM2',b'TIM4'),
                        (b'TIM_OC1',b'TIM_OC4'),(b'TIM_SetCompare1',b'TIM_SetCompare4')]:
        assert before in b,before;b=b.replace(before,after)
    assert b'    TIM_DeInit(TIM4);' in b
    b=b.replace(b'    TIM_DeInit(TIM4);',
                b'    /* PB9/CH4 shares TIM4 20ms with PB6/CH1 and PB7/CH2. No timer reset. */')
    b=b.replace(b'        TIM_GenerateEvent(TIM4, TIM_EventSource_Update);',
      newline.join([b'        if ((TIM4->CR1 & TIM_CR1_CEN) == 0U)',b'            TIM_GenerateEvent(TIM4, TIM_EventSource_Update);',b'        TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Enable);']))
    # Stop this output only. Stopping TIM4 would stop both radar servos.
    assert b'        TIM_Cmd(TIM4, DISABLE);' in b
    b=b.replace(b'        TIM_Cmd(TIM4, DISABLE);',b'        TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Disable);')
    return b

def main():
    assert not P.exists(),'Preserve an existing candidate'
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    path=P/'HARDWARE/GRIPPER/gripper.c'
    path.write_bytes(patch(path.read_bytes()))
    print(P)

if __name__=='__main__':main()
