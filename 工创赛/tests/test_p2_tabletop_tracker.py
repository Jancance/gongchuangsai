import unittest
from scripts.p2_tabletop_tracker import TabletopTracker,red_transition

def item(x=100,color='red'):
    return {'label':color,'silhouette_center_px':[x,100],
            'bbox_px':[x-20,80,40,40],'area_px2':1600}

class TabletopTrackerTests(unittest.TestCase):
    def test_continuous_observation_keeps_id(self):
        t=TabletopTracker()
        first=t.update([item()],1)['observed'][0]
        t.update([item(110)],1.07)
        last=t.update([item(120)],1.14)['observed'][0]
        self.assertEqual(first['id'],last['id']);self.assertEqual(last['state'],'OBSERVED')
    def test_short_absence_hides_then_reappears_unverified(self):
        t=TabletopTracker();old=t.update([item()],1)['observed'][0]['id']
        gap=t.update([],1.07)
        self.assertEqual(gap['observed'],[])
        self.assertEqual(gap['held_ids'],[old])
        self.assertEqual(gap['retired_ids'],[])
        self.assertEqual(t.visible(1.08),[])
        current=t.update([item()],1.14)['observed'][0]
        self.assertEqual(current['id'],old)
        self.assertEqual(current['state'],'REACQUIRED_UNVERIFIED')
        self.assertTrue(current['identity_continuity_unverified'])
        t.update([item()],1.21)
        self.assertEqual(t.update([item()],1.28)['observed'][0]['state'],'OBSERVED')

    def test_long_absence_retires_then_new_id(self):
        t=TabletopTracker();old=t.update([item()],1)['observed'][0]['id']
        t.update([],1.07)
        self.assertEqual(t.expire(1.51),[old])
        self.assertNotEqual(t.update([item()],1.58)['observed'][0]['id'],old)
    def test_duplicate_color_ambiguous(self):
        t=TabletopTracker();t.update([item()],1)
        r=t.update([item(100),item(180)],1.07)
        self.assertEqual(r['observed'],[]);self.assertEqual(r['reason'],'DUPLICATE_COLOR_REJECTED')
    def test_stale_and_old_timestamp_clear(self):
        t=TabletopTracker();t.update([item()],1)
        self.assertEqual(t.visible(1.16),[])
        r=t.update([item()],.9);self.assertEqual(r['reason'],'NONINCREASING_TIMESTAMP')
    def test_large_jump_and_color_change_create_new_id(self):
        t=TabletopTracker();one=t.update([item()],1)['observed'][0]['id']
        two=t.update([item(500)],1.07)['observed'][0]['id']
        three=t.update([item(500,'blue')],1.14)['observed'][0]['id']
        self.assertEqual(len({one,two,three}),3)

    def test_first_red_loss_and_id_change_trigger_diagnostic(self):
        red=lambda identity:{'observed':[{'id':identity,'color':'red'}]}
        self.assertEqual(red_transition(red('T002'),{'observed':[]}), 'first-red-loss')
        self.assertEqual(red_transition(red('T002'),red('T003')), 'first-red-id-change')
        self.assertEqual(red_transition({'held_ids':['T002'],'observed':[]},red('T002')),
                         'first-red-provisional-reacquisition')
        self.assertIsNone(red_transition(red('T002'),red('T002')))
        self.assertIsNone(red_transition({'observed':[]},red('T003')))

if __name__=='__main__':unittest.main()
