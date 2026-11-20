"""Add independent boundary cases to the actual Cortex-M4 CAN replay."""
from pathlib import Path

OWNER_RUNNER = Path(__file__).with_name("can_reads_arm.py")
runner = OWNER_RUNNER.read_text(encoding="utf-8")
original = "namespace['UNIT'] = unit + f6_vectors + VECTORS.read_text(encoding='ascii')"
assert runner.count(original) == 1
runner = runner.replace("can_reads_arm_", "can_reads_independent_")
runner = runner.replace(original, """
unit = replace(unit,
    'static uint32_t read_case, read_visible[8];',
    'static uint32_t read_case, read_visible[8], inject_diagnostic_preempt;')
unit = replace(unit,
    'sample_result=r;last_sample=*s;',
    'sample_result=r;last_sample=*s;if(inject_diagnostic_preempt)test_tick++;')
vectors = VECTORS.read_text(encoding='ascii')
vectors = replace(vectors, 'READ_BAD_TRAILER, READ_BAD_DLC',
    'READ_BAD_TRAILER, READ_BAD_DLC, READ_SPLIT')
vectors = replace(vectors, 'if (read_case==READ_PARTIAL) rx_count--;', '''
    if (read_case==READ_SPLIT) {
        uint8_t tail=rx_queue[3].Data[7];
        rx_queue[3].DLC--;
        rx_queue[5]=rx_queue[4];
        rx_queue[5].ExtId++;
        rx_queue[4].Data[1]=tail;
        rx_count=6U;
        for(i=0;i<rx_count;i++)read_visible[i]=first_part_ms-2U+i;
    }
    if (read_case==READ_PARTIAL) rx_count--;''')
vectors = replace(vectors, '    speed_cases();', '''
    speed_cases();
    /* At wrap, timestamp zero can be a successful observation. */
    read_begin(READ_OK,0xfffffffaU,2U,6U);
    can_read_case(0U,READ_OK,0xfffffffaU,2U,6U,1U);
    read_begin(READ_OK,0xfffffffaU,2U,6U);
    can_read_case(1U,READ_OK,0xfffffffaU,2U,6U,1U);

    /* Simulate preemption during snapshot logging after a timely final CAN frame. */
    read_begin(READ_OK,100U,2U,6U);
    inject_diagnostic_preempt=1U;
    {
        uint32_t result=CAN1_ReadEmmSystemFeedbackBounded(2U,100U,&reply,&feedback);
        ck(test_tick==107U,6100U);
        ck(result!=1U && !feedback.valid,6101U);
    }
    inject_diagnostic_preempt=0U;

    /* A six-fragment split still reconstructs the correct 30-byte payload. */
    read_begin(READ_SPLIT,100U,0U,6U);
    {
        uint32_t result=CAN1_ReadEmmSystemFeedbackBounded(2U,100U,&reply,&feedback);
        ck(result!=1U && !feedback.valid,6201U);
    }
    ''')
namespace['UNIT'] = unit + f6_vectors + vectors
""")
exec(compile(runner, str(OWNER_RUNNER), "exec"),
     {"__file__": str(OWNER_RUNNER), "__name__": "__main__"})
